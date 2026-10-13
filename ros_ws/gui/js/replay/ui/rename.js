/**
 * rename.js — the one client of POST /api/replay/recordings/<id>/rename (picker rows and the replay dock).
 *
 * The backend renames the folder, the .mcap and metadata.yaml to '<timestamp>_<name>' (a blank name restores the
 * bare timestamp) and answers {id: <new id>}. Rejects with an Error whose message is the server's reason text.
 */

/**
 * @param {function} fetchFn
 * @param {string} id the recording's current id
 * @param {string} name the user's text (the server makes it filename-safe)
 * @param {string} [base] '/api/replay'
 * @returns {Promise<string>} the new id
 */
export async function postRename(fetchFn, id, name, base) {
    const url = (base === undefined ? '/api/replay' : base) + '/recordings/' + encodeURIComponent(id) + '/rename';
    const res = await fetchFn(url, {
        method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ name: name || '' }),
    });
    let j = {};
    try { j = await res.json(); } catch (e) { j = {}; }
    if (!res.ok) throw new Error(j.error || ('HTTP ' + res.status));
    return j.id;
}
