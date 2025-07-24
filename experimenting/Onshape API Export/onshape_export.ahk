#Requires AutoHotkey v2.0
#SingleInstance Force
SetTitleMatchMode 2          ; not strictly needed now but harmless

; ---- helper: copy active tab’s URL without disturbing clipboard ----
CopyActiveURL() {
    ClipSaved := ClipboardAll()
    Send "^l"                 ; focus address bar
    Sleep 40
    Send "^c"                 ; copy
    ClipWait 2
    url := A_Clipboard
    Send "{Escape}"
    A_Clipboard := ClipSaved
    return url
}

; ---- hotkey: Win + Ctrl + p ----
#^p:: {
    url := CopyActiveURL()
    if !RegExMatch(
        url
      , "https://cad\.onshape\.com/documents/([0-9a-f]+)/[wv]/([0-9a-f]+)/e/([0-9a-f]+)"
      , &m)
        return

    did := m[1], wid := m[2], eid := m[3]

    pythonExe  := "C:\Python311\python.exe"
    scriptPath := "E:\AutoHotKey_Scripts\onshape_export_parts.py"
    params     := '"' scriptPath '" ' did ' ' wid ' ' eid

    exitCode := RunWait(pythonExe " " params, , "Hide")   ; wait for exporter

    if (exitCode = 0) {
        TrayTip("Onshape export"
              , "STEP saved to Downloads")
    } else {
        TrayTip("Onshape export"
              , "Export failed (code " exitCode ")")
    }
}