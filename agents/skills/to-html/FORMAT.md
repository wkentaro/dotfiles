# HTML Format

The explanation is rendered as a single self-contained HTML file. Tailwind and
Mermaid both come from CDNs. Mermaid handles graph-shaped diagrams reliably;
hand-built divs and inline SVG handle the more editorial visuals. Mix the two.
Do not lean on Mermaid for everything, it starts to look generic.

## Scaffold

```html
<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8" />
    <title>{{short title for the content}}</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <script type="module">
      import mermaid from "https://cdn.jsdelivr.net/npm/mermaid@11/dist/mermaid.esm.min.mjs";
      mermaid.initialize({ startOnLoad: true, theme: "neutral", securityLevel: "loose" });
    </script>
    <style>
      /* small custom layer for things Tailwind does not cover cleanly:
         dashed seam lines, hand-drawn-feeling arrow heads, etc. */
      .dashed { stroke-dasharray: 4 4; }
      .bad { stroke: #dc2626; }
      .accent { background: linear-gradient(135deg, #0f172a, #1e293b); }
    </style>
  </head>
  <body class="bg-stone-50 text-slate-900 font-sans">
    <main class="max-w-5xl mx-auto px-6 py-12 space-y-12">
      <header>...</header>
      <section class="space-y-10">...</section>
    </main>
  </body>
</html>
```

## Header

Title, date, and a compact one-line legend if the document uses a visual
convention (color or line meaning). No throat-clearing introduction paragraph.
Go straight into the content.

## Sections

The diagrams carry the weight. Prose is sparse and plain.

Group the content into `<section>`s, each with a short heading. Inside a
section, a typical unit is one `<article>`:

- **Title** - short, names the thing.
- **Badge row** (optional) - a status or category tag. Use color with meaning:
  emerald = good/strong, amber = caution, slate = neutral, red = problem.
- **The carrying visual** - a diagram, comparison, or before/after. This is the
  centerpiece, not decoration.
- **One or two sentences** - what it is, why it matters. Not paragraphs.
- **Bullets** when listing - keep each to a handful of words.

No paragraphs of explanation. If a diagram needs a paragraph to be understood,
redraw the diagram.

## Diagram patterns

Pick what fits. Mix them. Do not make every diagram look the same, variety is
part of the point.

### Mermaid graph (the workhorse for flow / relationships)

Use a Mermaid `flowchart` or `graph` when the point is "X leads to Y leads to Z."
Wrap it in a Tailwind-styled card so it does not feel parachuted in. Use
`classDef` to color meaningful edges or nodes. Sequence diagrams work well for
"before: 6 round-trips; after: 1."

```html
<div class="rounded-lg border border-slate-200 bg-white p-4">
  <pre class="mermaid">
    flowchart LR
      A[Input] --> B[Step]
      B --> C[Result]
      C -.problem.-> D[Failure]
      classDef bad stroke:#dc2626,stroke-width:2px;
      class C,D bad
  </pre>
</div>
```

### Hand-built boxes-and-arrows (when Mermaid's layout fights you)

Boxes as `<div>`s with borders and labels. Arrows as inline SVG `<line>` or
`<path>` positioned absolutely over a relative container. Reach for this when you
want a specific emphasis Mermaid will not give the right weight.

### Before / After (two columns)

Two columns side by side, same scale, so the change reads at a glance. Good for
"this is what is happening" vs "this is the fix."

### Cross-section (stacked bands)

Stack horizontal bands (`h-12 border-l-4`) to show stages or layers something
passes through. Useful for pipelines, request paths, or step counts.

### Comparison matrix / table

A clean Tailwind table when the content is genuinely tabular (options vs
criteria). Keep it light: thin borders, lots of padding, one accent for the
winning cell or row.

## Style guidance

- Lean editorial, not corporate-dashboard. Generous whitespace. Serif optional
  for headings (`font-serif` works well with stone/slate).
- Color sparingly: one accent (emerald or indigo) plus red for problems and
  amber for warnings.
- Keep diagrams around 320px tall so before/after sits comfortably side by side
  without scrolling.
- Use `text-xs uppercase tracking-wider` for schematic labels inside diagrams,
  so they read as diagram labels, not as UI chrome.
- The only scripts are the Tailwind CDN and the Mermaid ESM import. The file is
  otherwise static. No app code, no interactivity beyond Mermaid's rendering.

## Tone

Plain English, concise. Concision is not an excuse to be vague. Say the thing,
show the diagram, move on.
