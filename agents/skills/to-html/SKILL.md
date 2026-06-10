---
name: to-html
description: Render an explanation, analysis, plan, findings doc, or draft as a single self-contained HTML file (generous whitespace, sparse prose, diagram-first, one accent color) the user can open, screenshot, share, or copy from. Use when the user wants any standalone HTML document regardless of domain, e.g. says "explain this as html", "turn this into html", "visualize with html", or "make an html doc/report so I can review/discuss/copy". Content-agnostic; carries a fixed visual aesthetic.
---

# /to-html - Render anything as a polished standalone HTML file

The user wants whatever you would have written as a chat reply turned into a
single, self-contained, good-looking HTML file. They typically want to open it,
screenshot it for a colleague, paste it somewhere, or read it as a document
instead of scrolling a terminal.

This skill is about the **output format and delivery**, not the content. The
content can be anything: an explanation of a concept, a bug and a fix plan, a
findings-and-options decision doc, a draft to copy. Do not impose any
domain-specific vocabulary on the content. The look is fixed; the substance is
whatever the user asked about.

## Workflow

### 1. Settle the content first

Know what you are explaining before you open an HTML tag. If the user pointed at
a diagram, a diff, a bug, an idea, gather and understand it the same way you
would for a normal reply. The HTML is a rendering of a clear answer, not a
substitute for having one.

### 2. Build the file per FORMAT.md

Read [FORMAT.md](FORMAT.md) and follow it. The short version:

- One self-contained `.html` file. The only external resources are the Tailwind
  CDN and the Mermaid ESM import. No build step, no app code.
- Editorial, not corporate-dashboard. Generous whitespace, sparse prose, one
  accent color, red for problems, amber for warnings.
- Diagrams carry the weight. Mix Mermaid with hand-built boxes/SVG so it does
  not look generic. If a diagram needs a paragraph to be understood, redraw it.

### 3. Write it where the user can get at it

- Inside a git repo: write to `tmp/<descriptive-name>.html` in the repo root
  (create `tmp/` if missing). Outside a repo: use the OS temp dir.
- Name it for the content (`cross-matrix-explainer.html`, `ticket-4309-findings.html`),
  not after this skill.
- Print the absolute path. Offer to open it with `xdg-open` (Linux) but do not
  open it unprompted.

### 4. Self-check, then stop

Before handing over, confirm: the file is self-contained (no external scripts or
assets beyond the Tailwind CDN and the Mermaid import), every section carries a
visual (diagram, comparison, or before/after), no prose paragraph stands alone
without a visual or bullets beside it, and it links to its sources and any
sibling or index docs that exist. Fix whatever fails.

This skill produces the artifact, it does not iterate to perfection on its own.
If the user wants it polished further, that is a separate pass (e.g. /impeccable).
