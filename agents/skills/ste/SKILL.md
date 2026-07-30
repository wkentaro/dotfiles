---
name: ste
description: Write or rewrite technical documentation to adhere to ASD-STE100 Simplified Technical English, which strips AI-slop tells (hedging, synonym variation, sprawling clauses) as a side effect. Use when the user prefixes a doc-writing request with /ste.
argument-hint: "<what to write, or paste text to rewrite>"
---

# Simplified Technical English (ASD-STE100)

Write (or rewrite) the target in `$ARGUMENTS` to adhere to ASD-STE100. Recall the
full standard; the checklist below is the promptable core, not the whole of it.

## Rules

- One instruction per sentence. One idea per sentence for descriptions.
- Active voice. Simple tenses only (present, past, future). No perfect, no progressive.
- Start each instruction with the command verb ("Close the valve", not "The valve should be closed").
- Keep the articles and relative pronouns ("the pump", "the wire that connects").
- One word, one meaning, one part of speech. Reuse the same term for the same thing; no elegant variation.
- Max ~20 words per instruction sentence, ~25 per descriptive sentence.
- Put a warning before the step it applies to.
- No slang, no jargon, no noun clusters longer than three words, no ing-noun for an action.

## Output

Return the STE version only. If a required word has no plain synonym, keep it and note
the substitution you would need in one line at the end.
