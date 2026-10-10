/**
 * dom.js — tiny element builder. Every ui module takes its `document` through a factory, so a node
 * harness can pass a minimal fake (createElement/appendChild/replaceChildren/addEventListener/classList).
 */

/**
 * @param {Document} doc
 * @param {string} tag
 * @param {{cls?:string, text?:string, id?:string, title?:string, attrs?:object, on?:object, hidden?:boolean}} [o]
 * @param {Array} [kids] elements (null/false entries skipped)
 */
export function h(doc, tag, o, kids) {
    const e = doc.createElement(tag);
    o = o || {};
    if (o.cls) e.className = o.cls;
    if (o.id) e.id = o.id;
    if (o.text !== undefined) e.textContent = o.text;
    if (o.title) e.title = o.title;
    if (o.hidden !== undefined) e.hidden = !!o.hidden;
    if (o.attrs) for (const k of Object.keys(o.attrs)) e.setAttribute(k, o.attrs[k]);
    if (o.on) for (const k of Object.keys(o.on)) e.addEventListener(k, o.on[k]);
    if (kids) for (const k of kids) if (k) e.appendChild(k);
    return e;
}

/** Replace all children of `el` with `kids`. */
export function fill(el, kids) {
    el.replaceChildren();
    for (const k of kids) if (k) el.appendChild(k);
}

/** getElementById, or create `tag#id` under `parent` when the page does not have it yet. */
export function ensure(doc, id, parent, tag, cls) {
    let e = doc.getElementById(id);
    if (!e) {
        e = doc.createElement(tag || 'div');
        e.id = id;
        if (cls) e.className = cls;
        parent.appendChild(e);
    }
    return e;
}
