# PROTOTYPE: TaskTree frontend on a thin file-backed server (ticket #24)

Throwaway. Answers one question for wayfinder map #21: does the STOCK
TaskTree frontend run usefully against a thin Python server that serves the
minimal contract from #22 out of markdown item files? Nothing here is
production; the fork (#31) starts from the verdict, not from this code.

## Run

Terminal 1 (this directory):

    python thin_server.py            # items/ on http://127.0.0.1:8765

Terminal 2 (the unmodified TaskTree clone, nothing committed there):

    cd E:/PDJ/Github/TaskTree/frontend
    VITE_API_PROXY_TARGET=http://localhost:8765 npm run dev:local

Open http://localhost:5173/ . The Vite dev proxy forwards /api to the stub,
so there is no CORS. On Ctrl-C the server prints the endpoints hit and the
unhandled paths: that is the glue count.

## Item files

One markdown file per map item in `items/`. Frontmatter keys: `kind`
(pathway | plan | phase | idea | feature | bug | task, shown as TaskTree's
"stage"), `title`, `parent` (slug), `status`, `priority`, `destination`
(shown as completion criteria), `owners`, `blocked_by` (slugs), `branch`,
`canvas` ({x, y, collapsed}), `rank`, `work_days`, `start_date`,
`due_date`, `id` (written on first save). Body = description. Edit a file
by hand and refresh the browser: the index is rebuilt from disk per request.
Every edit in the UI is written straight back to the file.

Seeded from the #26 answer: five pathways, their plans, the oval's five
phases, three (b)-residue bugs and the three seed Ideas.
