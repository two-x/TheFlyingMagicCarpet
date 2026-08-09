# Claude Code standing prompts for this repo

Four copy-pasteable prompts. Each one, pasted into a Claude Code session
working in this repo (`CarpetLightCode`), sets up a **standing instruction**
that applies to every future response in that session until you tell it
otherwise — not a one-time task. Paste any subset of the 4; they're
independent of each other.

They're written to be self-contained: a fresh Claude session with no other
context should be able to follow one correctly. You shouldn't need to edit
these prompts over time — only if you specifically want to change *what
counts as in scope* for one of them (e.g. "also update X when Y changes").

## How to use these

1. Pick the prompt(s) you want (see below).
2. Paste the whole fenced block as a single message.
3. Claude keeps doing this at the end of every response for the rest of the
   session (or until you say "stop doing that" / start a new session).

You can paste more than one in the same message if you want several of
these running at once.

---

## 1. Keep README.md's firmware description in sync

Use this if you're actively changing firmware behavior (`src/*.h`,
`src/*.cpp`) and want the repo's `README.md` to stay accurate without a
separate manual doc pass.

```text
Upon each prompt answer from now on, until further notice, for each change
currently staged in git: if any staged file under src/ (firmware source —
.h/.cpp files) changes something README.md documents or should document
(fixture behavior, brightness math, button/config-mode behavior, per-show
logic, AudioBoard's audio-processing pipeline, defaults, persisted Nvm
settings, etc.), update README.md's relevant section(s) to match before
your response ends.

Scope: only touch the section(s) actually affected by what's staged right
now. Don't do a full unrelated audit of the rest of the file each time —
but if you're editing a paragraph and notice it references a stale
API/getter name or wrong default because of an EARLIER unrelated change,
fix that too rather than leaving it wrong next to your own edit.

Formatting/style — match the file's existing conventions exactly:
- One `#` H1 at the very top only. Top-level sections are `##`, subsections
  `###`. Find the right section by its heading text/content, don't assume
  fixed line numbers (the file changes shape over time).
- Prose is dense technical documentation, not marketing copy or a tutorial:
  third person, explains WHY (a design rationale, a bug that motivated a
  fix, a hardware constraint) as often as WHAT. No emoji.
- Bugfixes get an inline **Bugfix**: bold lead-in within the relevant
  paragraph, not a separate changelog section.
- Wrap identifiers, filenames, and function/getter names in single
  backticks: `AudioBoard::getBandHitPercent()`, `src/AudioBoard.h`.
- Use **bold** for defaults/key values inline (e.g. "default **68%**"), not
  for whole sentences.
- Use a `| --- |` markdown table for structured index/address/mapping data
  (DMX addresses, fixture directions, config subsettings) rather than a
  bullet list of the same thing.
- Cross-reference other sections in parens, e.g. "(see 'Configuration
  mode' above)" — don't duplicate content that already lives elsewhere in
  the file.

IMPORTANT: tools/visualizer/carpet-visualizer.html embeds a byte-for-byte
copy of this file's markdown source inside a
`<script type="text/markdown" id="fwReadmeSrc">` element (so its own "FW
readme" popup can render it without needing a server). Any time you change
README.md under this instruction, ALSO replace that embedded copy with the
new README.md content, verbatim, in the same response — the two must never
drift apart. (This is separate from prompt 2 below, which covers that
file's own *visualizer-specific* help content, not this embedded copy.)

End your response by explicitly noting "README.md updated: <1-line summary
of what changed>" or "README.md: no update needed this time" so it's clear
you checked.
```

## 2. Keep the visualizer's own Help content in sync

Use this if you're changing the visualizer (`tools/visualizer/carpet-visualizer.html`)
itself, OR changing firmware behavior that the visualizer's Help popup
documents (its config-menu map, its "inputs available to a show" getter
reference, etc).

```text
Upon each prompt answer from now on, until further notice, for each change
currently staged in git: if any staged change affects content covered by
the visualizer's own in-app "Visualizer Help" popup
(tools/visualizer/carpet-visualizer.html, inside
<div class="help-overlay" id="helpOverlay"> ... <div class="help-modal__body">),
update that HTML content to match before your response ends.

What this popup covers (and what counts as in scope): the VISUALIZER TOOL
itself — its architecture (real firmware compiled to WASM vs. simulated
external input), which controls are real FW I/O vs. a visualizer-only
shortcut vs. simulated audio, the firmware's config-menu screen/subsetting
map as the visualizer presents it, the show/variation list, and the
developer section (repo links, the getter API reference a new light show
would use, the cheat-sheet-card generator, the WASM rebuild step). It does
NOT cover general hardware/fixture documentation unrelated to the
visualizer — that's README.md (prompt 1), reachable from this same tool via
its separate "FW readme" popup/menu item. Don't duplicate README content
here; cross-reference it instead.

Formatting — match the existing markup exactly:
- Content is structured as numbered `<h3 id="help-slug">N. Title</h3>`
  sections. The `<div class="help-toc">` block near the top of the modal
  has one `<a href="#help-slug">N. Title</a>` per section — if you add,
  remove, or reorder a top-level section, update the TOC list and every
  section's number to match, in the same edit.
- Subsections inside a numbered section are `<h4>` (no numbering/anchor).
- Wrap identifiers/filenames/getter names in `<code>...</code>`.
- Where you're describing whether something is real firmware I/O, a
  visualizer-only dev convenience, or simulated external hardware, use the
  existing tag convention: `<span class="help-tag help-tag--fw">FW</span>`,
  `help-tag--shortcut` / `SHORTCUT`, `help-tag--sim` / `SIM` — see section 3
  ("What's real FW vs. a visualizer shortcut vs. simulated audio") for the
  definition of each.
- The "Inputs available to a show" list (section 6, for developers) is a
  getter-name reference for AudioBoard.h — if any getter is renamed, added,
  or its behavior changes (e.g. hit-decay timing, AGC modes, auto-peak),
  update this list's method names/description to match exactly, since a
  stale getter name here actively misleads a future show author.

End your response by explicitly noting "Visualizer help updated: <1-line
summary>" or "Visualizer help: no update needed this time" so it's clear
you checked.
```

## 3. Regenerate the control-box cheat sheet card

Use this if you're changing anything the printable control-box sticker
(`cheat_sheet_card.txt`/`.docx`, generated by `tools/gen_cheat_sheet_card.py`)
covers: the show/variation list or order, the config menu's top-level
screens or subsettings, or any button/pot/encoder behavior.

```text
Upon each prompt answer from now on, until further notice, for each change
currently staged in git: if any staged change affects the show/variation
list or order, the config menu's top-level screens or subsettings (in
CarpetLightLogic.cpp), or the global button/pot/encoder behavior map, then
update tools/gen_cheat_sheet_card.py's CELLS content to match, then run
`python3 tools/gen_cheat_sheet_card.py` (requires `pip install python-docx`
if not already installed) to regenerate cheat_sheet_card.txt and
cheat_sheet_card.docx at the repo root, before your response ends.

Do this by re-reading src/CarpetLightLogic.cpp, src/*Show.h, and src/Nvm.h
to confirm the CURRENT show/variation list and order, the config menu's
top-level modes and each mode's subsettings (in order), and each
show/variation's pot and encoder role plus the global button map — don't
assume anything staged already tells you the full current picture, since
CELLS has to reflect the post-change reality as a whole, not just the diff.

Keep the existing abbreviation style, column grouping, and print specs
unchanged: 6x1.5in page, ~1mm margins, Arial 10pt, 6 columns (see the
RECOMMENDED_FONT_PT/RECOMMENDED_COLUMNS/PAGE_W_IN/PAGE_H_IN constants and
the CELLS list itself in that script for the exact existing conventions —
match them, don't redesign the layout). If new content doesn't fit, trim
the least-essential detail (exact ms ranges, secondary parentheticals)
before shrinking anything else. Edit CELLS in the script — never hand-edit
the generated .txt/.docx files directly, they get overwritten.

End your response by explicitly noting "Cheat sheet card regenerated:
<1-line summary of what changed>" or "Cheat sheet card: no update needed
this time" so it's clear you checked.
```

## 4. Always end with a ready-to-paste git add/commit line

Use this any time you want Claude to stop short of actually committing, but
still hand you an exact command reflecting everything it just changed.

```text
Upon each prompt answer from now on, until further notice: end every
response (as long as any file was created, edited, staged, or otherwise
changed during it) with a single ready-to-paste shell line in this exact
form:

git add . <any-new-filenames>; git commit -m "[lightbox] 1) ... 2) ..."

- List each genuinely distinct change as its own numbered item (1), 2), 3)
  ...) inside the -m string — not one item per file, one item per
  logical/behavioral change, even if it touched several files.
- Keep each numbered item to one concise sentence/clause.
- "<any-new-filenames>" is literal instruction to you, not something to
  print verbatim: if `git status` shows any untracked (new) files this
  response created, list them explicitly after the "." (e.g.
  `git add . tools/new_thing.py;`); if there are no untracked files, just
  use `git add .` alone.
- Do NOT actually run `git commit` yourself — only ever print this line for
  the user to copy/paste and run themselves. Running `git add` yourself
  first (to check what's untracked) is fine; running the commit is not.
- If literally nothing changed in a given response (a pure question/answer
  with no file edits), skip this line entirely rather than printing an
  empty/no-op one.
```

---

## Notes for whoever's maintaining these

- These 4 prompts were written together in one session and cross-reference
  each other's territory (README vs. visualizer-help vs. cheat-sheet each
  have a defined, non-overlapping scope — see each prompt's own "what
  counts as in scope" paragraph) specifically so a dev can adopt any subset
  without gaps or duplicate work.
- If the visualizer's help-modal markup structure changes shape (e.g. the
  TOC/section-numbering convention changes), prompt 2 needs a matching
  update. Same for prompt 3 if `tools/gen_cheat_sheet_card.py`'s constants
  are renamed/restructured, or prompt 1 if README.md's own house style
  changes.
- Prompt 4's `[lightbox]` commit-tag prefix matches this repo's existing
  git history convention (see `git log`) — if that convention ever changes,
  update the literal string in prompt 4 to match.
