# Claude Code standing prompts for this repo

Two copy-pasteable prompts, the `CANTroller2` (control code) counterpart to
[`CarpetLightCode/claude_dev_prompts.md`](../CarpetLightCode/claude_dev_prompts.md)
(that file has the full 5-prompt set for the lightbox side, including a
self-maintaining prompt 5 — this file intentionally only has the 2 prompts
this codebase needs; see that file's own doc for the general pattern these
follow). Each one, pasted into a Claude Code session working in this repo,
sets up a **standing instruction** that applies to every future response in
that session until you tell it otherwise — not a one-time task.

They're written to be self-contained: a fresh Claude session with no other
context should be able to follow one correctly, reproducing the same
content, formatting, and conventions (including terseness and the table of
contents) this file's README currently has.

## How to use these

1. Pick the prompt(s) you want (see below).
2. Paste the whole fenced block as a single message.
3. Claude keeps doing this at the end of every response for the rest of the
   session (or until you say "stop doing that" / start a new session).

You can paste both in the same message if you want them running together.

---

## 1. Keep README.md in sync

```text
Upon each prompt answer from now on, until further notice, for each change
currently staged in git: if any staged file under src/ (firmware source —
.h/.cpp files) changes something README.md documents or should document
(sensor/motor behavior, the runmode state machine, safety interlocks, the
I2C/LightingBox protocol, display/UI structure, NeoPixel behavior,
temperature monitoring, diagnostics, persisted NVS settings, pin
assignments, etc.), update README.md's relevant section(s) to match before
your response ends.

Scope: only touch the section(s) actually affected by what's staged right
now. Don't do a full unrelated audit of the rest of the file each time —
but if you're editing a paragraph and notice it references something now
stale (a renamed function, a changed default, a wrong pin number) because
of an EARLIER unrelated change, fix that too rather than leaving it wrong
next to your own edit.

Formatting/style — match the file's existing conventions exactly:
- One `#` H1 at the very top only. Top-level sections are `##`, no `###`
  subsections currently exist but the same rule would apply if any get
  added. Find the right section by its heading text/content, don't assume
  fixed line numbers (the file changes shape over time).
- Prose is dense technical documentation, not marketing copy or a
  tutorial: third person, explains WHY (a design rationale, a bug that
  motivated a fix, a hardware constraint) as often as WHAT. No emoji.
- **Condensed, not padded.** Every fact, number, name, and rationale that's
  currently in a section must survive an edit to it — but cut filler:
  redundant restatement of something the previous sentence already said,
  throat-clearing, and words that don't carry information. A rough
  target: if you can cut 15-25% of a paragraph's word count without
  losing a single fact, do it. This is a standing property of the file —
  don't let new prose you add regress to a looser style than what's
  already there.
- Bugfixes/known-quirks get called out inline (e.g. "a documented
  historical bug:", "a known limitation:") within the relevant paragraph,
  not a separate changelog section — this codebase's own source comments
  already follow this style heavily (grep for phrases like "bug fix" or
  "previously" in src/*.h for the existing voice to match).
- Wrap identifiers, filenames, and class/function names in single
  backticks: `BrakeControl::carstop()`, `src/motors.h`.
- Use **bold** for defaults/key values inline (e.g. "default **68%**"), not
  for whole sentences.
- Use a `| --- |` markdown table for structured data (the pin map, the
  runmode table, the temperature-limits table) rather than a bullet list
  of the same thing.
- Cross-reference other sections with a markdown link to their heading
  anchor, e.g. `(see [Diagnostics and safety](#diagnostics-and-safety))`
  — don't duplicate content that already lives elsewhere in the file.
- **Headings stay anchor-friendly.** Keep them short, plain text — avoid
  packing a heading with an em-dash clause, a parenthetical file path,
  commas, or backticked code. If a detail like a filename would otherwise
  go in the heading, put it in that section's first sentence instead.
  This isn't cosmetic: the `## Contents` TOC's links (below) depend on
  each heading's auto-generated GitHub anchor slug staying short,
  predictable, and stable.

**Table of contents — keep it current.** Immediately under the file's
top intro/cross-reference paragraphs (before the first `##` section) is a
`## Contents` section: a markdown list of every `##` (and, if any exist,
nested `###`) heading in the file, in order, `###` nested one level under
its parent `##`. Each entry is `[Heading text](#slug)`, where `slug` is
the heading text lowercased, anything that isn't a letter/digit/space/
hyphen stripped out, then spaces turned into hyphens (e.g. "I2C bus and
the LightingBox link" → `#i2c-bus-and-the-lightingbox-link`) — the same
algorithm GitHub's own heading-anchor renderer uses for plain-text
headings, so these links work natively on github.com. Any time you add,
remove, rename, or reorder a heading, update the `## Contents` list to
match in the same edit — a stale TOC entry (wrong text, dead link, missing
entry) is as much a bug as a wrong technical fact.

Note what this file does NOT need (unlike its CarpetLightCode counterpart):
there's no visualizer here, so no embedded-markdown-copy to resync and no
second "what's real firmware vs a dev shortcut" content to keep parallel —
just this one README.

End your response by explicitly noting "README.md updated: <1-line summary
of what changed>" or "README.md: no update needed this time" so it's clear
you checked.
```

## 2. Always end with a ready-to-paste git add/commit line

Use this any time you want Claude to stop short of actually committing, but
still hand you an exact command (or commands) reflecting everything it just
changed. Written for this specific repo's shape: a single git repository
rooted at `~/Documents/TheFlyingMagicCarpet` containing two independent
codebases as sibling subdirectories — `CANTroller2/` (this one, "control
code," commit prefix `[control]`) and `CarpetLightCode/` (the light show
firmware, commit prefix `[lightbox]`). A commit must never contain changes
from both — this rule applies regardless of which codebase's directory the
current Claude Code session happens to be working in.

```text
Upon each prompt answer from now on, until further notice: end every
response (as long as any file was created, edited, staged, or otherwise
changed during it, anywhere in the ~/Documents/TheFlyingMagicCarpet repo)
with one or more ready-to-paste shell lines reflecting what changed,
following the rules below.

This repo has two independent codebases living side by side in one git
repo: CANTroller2/ ("control code," commit prefix [control]) and
CarpetLightCode/ ("lightbox code," commit prefix [lightbox]). Before
producing any commit line, check (e.g. via `git status --short` from the
repo root) which of the two subtrees actually have changes right now —
not just what this one response touched, but everything currently
unstaged/untracked in the repo, since a stray leftover change in the
other codebase must not get silently swept into this response's commit
either.

- **If changes exist in only one of the two subtrees**: print a single
  block in this exact form:

  git add CANTroller2/path/to/changed1.h CANTroller2/path/to/changed2.cpp; git commit -m "[control] 1) Section: detail 2) Section: detail"

  (or the `CarpetLightCode/`/`[lightbox]` equivalent). The `git add`
  argument list must be the **actual specific changed/new file paths**
  (from `git status`), each one explicitly prefixed with that one
  codebase's directory — never a bare `git add .`, and never a
  directory-level `git add CANTroller2/` either, since either form
  silently sweeps in whatever else happens to be sitting unstaged in that
  subtree (a leftover WIP change unrelated to this response) along with
  what you actually meant to commit. List every file precisely, every
  time.

- **If changes exist in BOTH subtrees**: say so explicitly in your
  response (e.g. "Both control and lightbox code changed — two separate
  commits needed"), then print TWO separate blocks back to back, one per
  codebase, each in the form above with that codebase's own prefix. Never
  merge them into one `git add`/`git commit` pair, even if it feels like
  one logical change spanning both — if the change genuinely spans both
  codebases, it still gets described from each codebase's own point of
  view in its own commit, split at the directory boundary.

- **Keep the whole message short.** List each genuinely distinct change as
  its own numbered item (1), 2), 3)...) inside a given -m string — one
  item per logical/behavioral change within THAT codebase, not one item
  per file. Each item is `Section: detail` — a few words naming the code
  section/subsystem/file the change is in (e.g. `BrakeControl`,
  `runmodes`, `README`, `Hotrc`), a colon, then a short, terse detail
  clause — a phrase, not a full sentence, and not the padded,
  rationale-heavy style used in this repo's own doc prose or in your chat
  responses. Compare:
  - Too long: "1) Reworked BrakeControl's hybrid feedback scheme so the
    sigmoid blend between position-dominant and pressure-dominant control
    now transitions between 25% and 50% of full pressure range instead of
    the old fixed 30% crossover point"
  - Right length: "1) BrakeControl: hybrid blend crossover now 25-50% of
    pressure range, was a fixed 30%"
- This applies equally to newly-created (untracked) files: include them by
  their exact path too, same as modified tracked files — `git status`
  shows both.
- Do NOT actually run `git commit` yourself — only ever print these lines
  for the user to copy/paste and run themselves. Running `git status`/
  `git add` yourself first (to check what's changed/untracked) is fine;
  running the commit is not.
- If literally nothing changed in a given response (a pure question/answer
  with no file edits, in either subtree), skip this entirely rather than
  printing an empty/no-op line.
- Edge case: a change to a file outside both `CANTroller2/` and
  `CarpetLightCode/` (e.g. the repo-root `README.md`, `.github/`) doesn't
  cleanly belong to either prefix — call this out explicitly rather than
  guessing which prefix to force it under, and ask which it should ride
  along with (or whether it needs its own commit) if it's not obvious
  from context.
```

---

## Notes for whoever's maintaining these

- This file and `CarpetLightCode/claude_dev_prompts.md` share one rule
  verbatim — the dual-codebase split-commit discipline in prompt 2 here /
  prompt 4 there. If that rule changes (a directory renamed, a 3rd
  codebase added to the repo, prefixes changed), update **both** files to
  match — neither is the sole source of truth for the other.
- `CarpetLightCode/claude_dev_prompts.md` also has a prompt 5 (a
  self-maintaining meta-prompt that keeps that file's own instructions in
  sync automatically). This file doesn't duplicate that prompt — if you
  want the same self-maintenance behavior for this file too, paste
  prompt 5 from the other file; its instructions already generalize to
  "whichever `claude_dev_prompts.md` the active session is working near."
- This file has no cheat-sheet or visualizer-help equivalent because this
  codebase doesn't have either of those artifacts — don't add sections for
  them speculatively; add a prompt here only when this codebase actually
  grows something that needs one.
