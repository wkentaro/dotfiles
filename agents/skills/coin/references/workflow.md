# The invent → screen Workflow script

Run this script verbatim via the Workflow tool. Everything run-specific
travels through `args` — never edit the script text itself; that is what
keeps the process identical across runs.

## args contract

```jsonc
{
  "qualities": "flow, precision, warmth",        // the framework's target experience qualities
  "framework": "<full text of framework.md>",
  "briefs": {
    "truth": "<real brief: product, category, framework>",
    "disguise": "<fictional adjacent-company brief — no real product/company>",
    "object": "<physical-object brief — no software, no category>",
    "mining": "<experience qualities + metaphor territories to mine>"
  },
  "widths": {
    "perLane": 250,        // directions per lane (quick 100 / default 250 / deep 250)
    "lanesPerRole": 1,     // deep: 2
    "triageKeep": 60,      // quick 30 / default 60 / deep 100
    "scoreKeep": 20,       // quick 10 / default 20 / deep 30
  },
  "linguisticsPath": "/abs/path/to/skills/coin/references/linguistics.md",
  "round2": null           // or, for round 2:
  // {
  //   "deltas": "<text of round-2/deltas.md>",
  //   "laneBriefs": ["<seeded brief 1>", "<seeded brief 2>"],
  //   "rescue": ["name", ...],          // near-misses from round-1 funnel
  //   "userCandidates": ["name", ...]   // co-creation entries
  // }
}
```

## Script

```js
export const meta = {
  name: 'coin-invent',
  description: 'Blind-lane name generation, triage, linguistic scoring, and screening',
  phases: [
    { title: 'Generate', detail: 'blind-briefed lanes produce directions' },
    { title: 'Triage', detail: 'dedup and promote the strongest candidates' },
    { title: 'Score', detail: 'rubric scoring against the bundled linguistics reference' },
    { title: 'Screen', detail: 'availability, collisions, trademark advisory, cultural panel' },
  ],
}

const A = typeof args === 'string' ? JSON.parse(args) : args
const W = A.widths
const QUALITIES = A.qualities

const DIRECTIONS = {
  type: 'object',
  properties: { directions: { type: 'array', items: { type: 'string' } } },
  required: ['directions'],
}
const CANDIDATES = {
  type: 'object',
  properties: {
    candidates: {
      type: 'array',
      items: {
        type: 'object',
        properties: {
          name: { type: 'string' },
          origin: { type: 'string' },
          note: { type: 'string' },
        },
        required: ['name'],
      },
    },
  },
  required: ['candidates'],
}
const SCORES = {
  type: 'object',
  properties: {
    scored: {
      type: 'array',
      items: {
        type: 'object',
        properties: {
          name: { type: 'string' },
          smile: {
            type: 'object',
            properties: {
              suggestive: { type: 'integer' }, memorable: { type: 'integer' },
              imagery: { type: 'integer' }, legs: { type: 'integer' },
              emotional: { type: 'integer' },
            },
            required: ['suggestive', 'memorable', 'imagery', 'legs', 'emotional'],
          },
          scratch: { type: 'array', items: { type: 'string' } },
          sound_note: { type: 'string' },
          fluency: { type: 'integer' },
          total: { type: 'number' },
        },
        required: ['name', 'smile', 'scratch', 'sound_note', 'fluency', 'total'],
      },
    },
  },
  required: ['scored'],
}
const RANKED = {
  type: 'object',
  properties: {
    selected: {
      type: 'array',
      items: {
        type: 'object',
        properties: { name: { type: 'string' }, why: { type: 'string' } },
        required: ['name', 'why'],
      },
    },
  },
  required: ['selected'],
}
const AVAIL = {
  type: 'object',
  properties: {
    domains: {
      type: 'object',
      properties: {
        com: { type: 'string' }, io: { type: 'string' },
        ai: { type: 'string' }, dev: { type: 'string' },
      },
      required: ['com', 'io', 'ai', 'dev'],
    },
    pypi: { type: 'string' },
    npm: { type: 'string' },
    github: { type: 'string' },
  },
  required: ['domains', 'pypi', 'npm', 'github'],
}
const COLLIDE = {
  type: 'object',
  properties: {
    collisions: { type: 'array', items: { type: 'string' } },
    trademark_advisory: { type: 'string' },
    severity: { type: 'string', enum: ['clear', 'flag', 'kill'] },
  },
  required: ['collisions', 'trademark_advisory', 'severity'],
}
const CULTURE = {
  type: 'object',
  properties: {
    languages: {
      type: 'array',
      items: {
        type: 'object',
        properties: {
          lang: { type: 'string' },
          verdict: { type: 'string', enum: ['ok', 'flag', 'kill'] },
          note: { type: 'string' },
        },
        required: ['lang', 'verdict'],
      },
    },
  },
  required: ['languages'],
}

const GEN_RULES = `You are one coining lane inside a naming pipeline. Produce exactly ${W.perLane} DIRECTIONS: single words, word fragments, two-word compounds, morpheme fusions, metaphors. Directions are raw material, not finished names. Suspend judgment entirely: volume and variety over quality; include wild, uncomfortable, and half-formed entries alongside safe ones. Vary length, sound, and register. Output only the list.`
const DIVERGENT = `\n\nTake an unconventional angle: favor entries the obvious approach would miss.`

phase('Generate')
let laneRuns
if (!A.round2) {
  laneRuns = [
    { role: 'truth', prompt: `${GEN_RULES}\n\nBRIEF (the real assignment):\n${A.briefs.truth}` },
    { role: 'disguise', prompt: `${GEN_RULES}\n\nBRIEF:\n${A.briefs.disguise}` },
    { role: 'object', prompt: `${GEN_RULES}\n\nBRIEF:\n${A.briefs.object}` },
    { role: 'mining', prompt: `You are the mining lane inside a naming pipeline. Produce exactly ${W.perLane} entries of RAW MATERIAL — real words, metaphors, sports, natural phenomena, tools, materials, textures, movements, and productive morphemes (prefixes, suffixes, roots) that evoke the qualities in the brief. Do NOT coin names; collect the vocabulary a name could be built from. Suspend judgment; range widely across domains. Output only the list.\n\nBRIEF:\n${A.briefs.mining}` },
  ]
  if (W.lanesPerRole > 1) {
    laneRuns = laneRuns.concat(laneRuns.map(l => ({ role: `${l.role}-b`, prompt: l.prompt + DIVERGENT })))
  }
} else {
  laneRuns = A.round2.laneBriefs.map((brief, i) => ({
    role: `round2-${i + 1}`,
    prompt: `${GEN_RULES}\n\nBRIEF:\n${brief}\n\nROUND-2 STEER (feedback from the first shortlist — weigh it heavily):\n${A.round2.deltas}`,
  }))
}
const laneOutputs = await parallel(laneRuns.map(l => () =>
  agent(l.prompt, { label: `lane:${l.role}`, phase: 'Generate', schema: DIRECTIONS })
))
const lanes = {}
laneRuns.forEach((l, i) => { lanes[l.role] = (laneOutputs[i] && laneOutputs[i].directions) || [] })

let synthesized = []
if (!A.round2) {
  const miningMaterial = Object.keys(lanes).filter(r => r.startsWith('mining')).flatMap(r => lanes[r])
  const synth = await agent(
    `You are the synthesis step of a naming pipeline. Below is raw vocabulary mined around these target qualities: ${QUALITIES}. Build ~80 candidate name directions from it: pair items into compounds, fuse morphemes, truncate, blend. The winning name is often sitting on a list like this. Suspend judgment; keep variety.\n\nRAW MATERIAL:\n${miningMaterial.join('\n')}`,
    { label: 'synthesize', phase: 'Generate', schema: CANDIDATES }
  )
  synthesized = ((synth && synth.candidates) || []).map(c => c.name)
}

phase('Triage')
const seen = new Set()
const pool = []
for (const role of Object.keys(lanes)) {
  if (role.startsWith('mining')) continue
  for (const d of lanes[role]) {
    const k = d.trim().toLowerCase()
    if (!k || seen.has(k)) continue
    seen.add(k)
    pool.push({ name: d.trim(), origin: role })
  }
}
for (const s of synthesized) {
  const k = s.trim().toLowerCase()
  if (!k || seen.has(k)) continue
  seen.add(k)
  pool.push({ name: s.trim(), origin: 'synthesis' })
}
const rawCount = Object.values(lanes).reduce((a, l) => a + l.length, 0) + synthesized.length
log(`${pool.length} unique directions from ${rawCount} raw`)

const CHUNK = 150
const chunks = []
for (let i = 0; i < pool.length; i += CHUNK) chunks.push(pool.slice(i, i + CHUNK))
const perChunkKeep = Math.max(5, Math.ceil((W.triageKeep * 2) / chunks.length))
const triageOut = await parallel(chunks.map((c, i) => () =>
  agent(
    `Triage step of a naming pipeline. Target qualities: ${QUALITIES}. From the directions below, promote the ${perChunkKeep} strongest entries that could become (or be shaped into) a product/company name serving those qualities. Convert a fragment into name form when the conversion is obvious. Favor distinctive over safe; this is promotion, not final judgment.\n\n${c.map(x => x.name).join('\n')}`,
    { label: `triage:${i + 1}`, phase: 'Triage', effort: 'low', schema: CANDIDATES }
  )
))
const promoted = triageOut.filter(Boolean).flatMap(t => t.candidates.map(c => c.name))
const merge = await agent(
  `Final triage merge of a naming pipeline. Target qualities: ${QUALITIES}. From the promoted candidates below, select exactly ${W.triageKeep} to advance to full linguistic scoring. Maximize variety of sound, length, and construction (real words, compounds, coined). Return each with a one-phrase note on its angle.\n\n${promoted.join('\n')}`,
  { label: 'triage:merge', phase: 'Triage', schema: CANDIDATES }
)
let advancing = ((merge && merge.candidates) || []).map(c => c.name)
log(`triage kept ${advancing.length} of ${pool.length} unique (${promoted.length} promoted by chunk triage)`)

if (A.round2) {
  const extra = (A.round2.rescue || []).concat(A.round2.userCandidates || [])
  const have = new Set(advancing.map(n => n.toLowerCase()))
  for (const e of extra) if (e.trim() && !have.has(e.trim().toLowerCase())) advancing.push(e.trim())
  log(`round 2: injected ${advancing.length - W.triageKeep} rescue/co-creation candidates`)
}

phase('Score')
const SCORE_CHUNK = 12
const scoreChunks = []
for (let i = 0; i < advancing.length; i += SCORE_CHUNK) scoreChunks.push(advancing.slice(i, i + SCORE_CHUNK))
const scoreOut = await parallel(scoreChunks.map((c, i) => () =>
  agent(
    `Scoring step of a naming pipeline. FIRST read the rubric file at ${A.linguisticsPath} — score only against it, not from general knowledge. Target qualities the name must evoke: ${QUALITIES}.\n\nFramework (including the competitive landscape):\n${A.framework}\n\nFor each name below give: SMILE subscores 0–2 each (suggestive, memorable, imagery, legs, emotional); scratch = list of SCRATCH veto flags that genuinely apply (empty when none) — apply "copycat" only against competitors named in the framework above; collisions with unrelated products are a later screening stage's job, not yours; sound_note = one phrase on how its sounds serve or fight the target qualities, citing the rubric's letter table; fluency 0–2 (processing fluency); total = SMILE sum + fluency (0–12).\n\nNAMES:\n${c.join('\n')}`,
    { label: `score:${i + 1}`, phase: 'Score', schema: SCORES }
  )
))
const scored = scoreOut.filter(Boolean).flatMap(s => s.scored)
const vetoed = scored.filter(s => s.scratch.length > 0)
const viable = scored.filter(s => s.scratch.length === 0)
log(`scored ${scored.length}; ${vetoed.length} SCRATCH-vetoed, ${viable.length} viable`)
const rank = await agent(
  `Ranking step of a naming pipeline. Target qualities: ${QUALITIES}.\n\nFramework:\n${A.framework}\n\nFrom the scored candidates below (JSON), select exactly ${Math.min(W.scoreKeep, viable.length)} to advance to screening. Scores calibrate you but do not bind you: prefer distinctive, polarizing names over comfortable ones — comfort is a warning sign. Keep construction variety. One sentence of why per pick.\n\n${JSON.stringify(viable)}`,
  { label: 'rank', phase: 'Score', schema: RANKED }
)
const finalists = ((rank && rank.selected) || [])
log(`advancing ${finalists.length} to screening`)

phase('Screen')
const screened = await parallel(finalists.map(f => () =>
  parallel([
    () => agent(
      `Availability check for the candidate name "${f.name}" (lowercase slug: ${f.name.toLowerCase().replace(/[^a-z0-9]/g, '')}). Use Bash only, no browsing. For each of ${f.name}.com/.io/.ai/.dev: run dig +short NS <domain>; NS records present => "taken", empty => "likely free". PyPI: curl -s -o /dev/null -w '%{http_code}' https://pypi.org/pypi/<slug>/json (404 => "free"). npm: same against https://registry.npmjs.org/<slug>. GitHub: curl -s -o /dev/null -w '%{http_code}' https://api.github.com/users/<slug> and note whether the org/user name is taken. Report exactly what you observed; when a check fails to run, say "unchecked", never guess.`,
      { label: `avail:${f.name}`, phase: 'Screen', effort: 'low', schema: AVAIL }
    ),
    () => agent(
      `Collision and trademark advisory for the candidate name "${f.name}", intended for a developer-tools / software product. Web-search for active products, companies, or open-source projects already using this or a confusingly similar name, especially in software; list real collisions found (empty when none). Then best-effort search of trademark registers (USPTO, EUIPO, J-PlatPat) via the web for live marks in software/SaaS classes. trademark_advisory must begin with "ADVISORY — not legal clearance:". severity: "kill" only for a direct same-space collision with an active product; "flag" for adjacent risk; else "clear".`,
      { label: `collide:${f.name}`, phase: 'Screen', schema: COLLIDE }
    ),
    () => agent(
      `Cultural screen for the candidate name "${f.name}" across exactly these languages: English, Japanese, Spanish, Portuguese, German, Mandarin. For each: does the name (spoken or written, including plausible transliterations — for Japanese use katakana rendering per the Japanese section of ${A.linguisticsPath}) collide with profanity, slang, mockery-inviting words, or dark associations? Verdict per language: "kill" for the terrible, "flag" for the worrisome, "ok" otherwise, with a short note when not ok.`,
      { label: `culture:${f.name}`, phase: 'Screen', schema: CULTURE }
    ),
  ]).then(checks => ({
    name: f.name,
    why: f.why,
    availability: checks[0],
    collision: checks[1],
    cultural: checks[2],
  }))
))
const results = screened.filter(Boolean)
const killed = results.filter(r =>
  (r.collision && r.collision.severity === 'kill') ||
  (r.cultural && r.cultural.languages.some(l => l.verdict === 'kill'))
)
log(`screening done: ${results.length} screened, ${killed.length} killed`)

return {
  funnel: {
    raw: rawCount,
    unique: pool.length,
    triaged: advancing.length,
    scratchVetoed: vetoed.map(v => ({ name: v.name, flags: v.scratch })),
    scoredViable: viable.length,
    screened: results.length,
    killedInScreening: killed.map(k => k.name),
  },
  lanes,
  synthesized,
  triaged: advancing,
  scored,
  finalists,
  screened: results,
}
```

## After the run

The script returns everything; write the workspace files from its return
value (`directions/<lane>.md` from `lanes` + `synthesized`, `funnel.md`
from `funnel` with every cut and its reason, `scores.md` from `scored`,
`screening.md` from `screened`). If a stage looks empty, read the run's
`journal.jsonl` before re-running — cached results may simply be empty.
