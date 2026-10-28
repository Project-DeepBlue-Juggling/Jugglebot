# -*- coding: utf-8 -*-
"""Static-markup contract for the bar's quick-capture "+" (ros_ws/gui/js/quick-capture.js)."""
from __future__ import annotations

from pathlib import Path

GUI = Path(__file__).resolve().parents[2] / "ros_ws" / "gui"
HTML = (GUI / "index.html").read_text()
JS = (GUI / "js" / "quick-capture.js").read_text()
CSS = (GUI / "css" / "app-tabs.css").read_text()


def test_button_lives_in_the_bar_and_form_exists_once():
    bar = HTML[HTML.index('id="app-tabs"'):HTML.index('id="map-pane"')]
    assert 'id="quick-capture-btn"' in bar
    assert 'title="File a bug or idea in the map"' in bar
    for ident in ("quick-capture-btn", "quick-capture-form", "qc-title", "qc-note", "qc-file", "qc-cancel"):
        assert HTML.count(f'id="{ident}"') == 1
    for kind in ("bug", "idea", "feature"):
        assert f'name="qc-kind" value="{kind}"' in HTML
    assert 'value="bug" checked' in HTML
    assert 'js/quick-capture.js' in HTML


def test_button_sits_left_right_of_the_map_tab_and_says_what_it_does():
    """JB-65 (owner): the control lives on the left, just right of the Map tab, not in the prefs group."""
    bar = HTML[HTML.index('id="app-tabs"'):HTML.index('id="map-pane"')]
    btn = bar.index('id="quick-capture-btn"')
    assert bar.index('id="tab-map"') < btn < bar.index('class="app-tabs-prefs"')
    assert '>+<span class="qc-label"> File new change request</span></button>' in bar
    assert 'white-space: nowrap' in CSS and '.qc-label { display: none; }' in CSS  # one line; "+" alone on a phone
    assert 'getBoundingClientRect' in JS  # the popover is placed under the button


def test_form_keys_never_reach_the_robot_tab():
    assert "form.addEventListener('keydown'" in JS
    assert "e.stopPropagation()" in JS
    assert "'Escape'" in JS


def test_ctrl_or_cmd_enter_files_from_any_control():
    # JB-78: the chord is handled in the form's own keydown (so it works from the kind
    # radios and the buttons too), calls the busy-guarded file(), and stops the
    # browser's implicit Enter-submit from filing a second time.
    i = JS.index("form.addEventListener('keydown'")
    handler = JS[i:JS.index('});', i)]
    assert "e.key === 'Enter' && (e.ctrlKey || e.metaKey)" in handler
    assert 'e.preventDefault(); file();' in handler
    assert 'if (!title || busy)' in JS


def test_focus_goes_to_title_and_returns_to_button():
    assert "titleEl.focus()" in JS and "btn.focus()" in JS


def test_post_path_header_and_inbox_status():
    assert "/projects/${encodeURIComponent(pid)}/tasks" in JS
    assert "'/projects'" in JS
    assert "'X-Dev-User'" in JS
    assert "status: 'inbox'" in JS
    assert "MAP_PORT = 8765" in JS and "'/api/v1'" in JS
    assert "credentials:" not in JS


def test_failure_keeps_text_and_draft_key_used():
    assert "Map server unreachable - text kept" in JS
    assert "'jugglebot.quickCapture.draft'" in JS
    assert "setItem(DRAFT_KEY" in JS and "removeItem(DRAFT_KEY" in JS and "getItem(DRAFT_KEY" in JS
    # the draft is cleared only on the success path, after the ok check
    assert JS.index("if (!resp.ok)") < JS.index("clearDraft();\n        titleEl.value")


def test_context_block_has_time():
    assert "Filed from the Robot tab at" in JS


def test_popover_styled_under_the_bar():
    assert "#quick-capture-form" in CSS and "top: var(--app-tabs-height)" in CSS


def test_plus_button_keys_never_reach_chart_pause_or_trackbar():
    assert "for (const type of ['keydown', 'keyup', 'keypress'])" in JS
    assert "btn.addEventListener(type, (e) => e.stopPropagation())" in JS


def test_user_dismissals_are_ignored_while_a_request_is_in_flight():
    # Esc, Cancel and the "+" toggle all go through dismiss(), which respects busy
    assert "function dismiss() { if (!busy) closeForm(); }" in JS
    assert "cancelBtn.addEventListener('click', dismiss)" in JS
    assert "if (e.key === 'Escape') { e.preventDefault(); dismiss(); }" in JS
    assert "else dismiss();" in JS
    # closeForm() is called only by dismiss and by the success path (plus its definition)
    assert JS.count("closeForm()") == 3
    assert JS.index("clearDraft();\n        titleEl.value") < JS.index("closeForm();\n        showToast(`Filed")


def test_error_goes_to_the_toast_when_the_form_is_hidden():
    assert "if (form.hidden) showToast(text);" in JS
    assert "else errEl.textContent = text;" in JS
    # no filing path writes the error element directly any more
    body = JS[JS.index("async function file()"):JS.index("btn.addEventListener('click'")]
    # (the "Title is required" validation stays direct: the form is necessarily visible there)
    assert "errEl.textContent = await" not in body
    assert "errEl.textContent = 'Map server" not in body
    assert body.count("showError(") == 4


def test_post_timeout_warns_the_item_may_exist():
    assert "No answer from the map server - check the inbox before filing again" in JS
    # only after the POST was sent, and only for the abort
    assert "posted && e && e.name === 'AbortError'" in JS
    assert JS.index("posted = true;") < JS.index("api(`/projects/${encodeURIComponent(pid)}/tasks`")


def test_click_outside_the_open_form_dismisses_it_without_filing():
    """JB-80: pointerdown outside form and toggle -> the busy-guarded dismiss(); never file()."""
    assert "document.addEventListener('pointerdown'" in JS
    i = JS.index("document.addEventListener('pointerdown'")
    line = JS[i:JS.index('\n', i)]
    assert "isOutsideClick(e.target)" in line and "dismiss()" in line and "file()" not in line
    assert line.rstrip().endswith("true);")  # capture phase
    fn = JS[JS.index("function isOutsideClick"):JS.index("function pad")]
    assert "!form.hidden" in fn and "!form.contains(target)" in fn and "!btn.contains(target)" in fn
    assert "function dismiss() { if (!busy) closeForm(); }" in JS
