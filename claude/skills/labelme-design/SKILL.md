# LabelMe Design System Skill

Use this skill when building or editing pages for **labelme.io**.
It ensures every page matches the visual language of the root page.

## Stack

- **CSS framework**: Tailwind CSS v4
- **Font**: Inter (loaded via `https://fonts.cdnfonts.com/css/inter`)
- **Icons**: Phosphor Icons v2.1.1 (regular, fill, duotone variants)
- **Max content width**: `max-w-[1280px]` (sections up to `max-w-[1380px]`)
- **Base spacing unit**: 0.25rem (Tailwind default)

---

## Color Palette

### Brand — Emerald (primary actions, badges, highlights)

| Token           | Usage                                   |
| --------------- | --------------------------------------- |
| `emerald-50`    | Section backgrounds, subtle tints       |
| `emerald-100`   | Badge backgrounds, feature card tints   |
| `emerald-200`   | Borders on cards/badges                 |
| `emerald-300`   | Gradient accents                        |
| `emerald-600`   | **Primary CTA buttons**, icon fills     |
| `emerald-700`   | Button hover state, strong text         |

### Accent — Orange (pricing, secondary CTAs, warnings)

| Token           | Usage                                   |
| --------------- | --------------------------------------- |
| `orange-50`     | Pricing section background              |
| `orange-100`    | Hover state on pricing cards            |
| `orange-200`    | Borders on pricing elements             |
| `orange-500`    | **Secondary CTA buttons**, price tags   |
| `orange-600`    | Hover state, emphasized price text      |
| `orange-700`    | Strong pricing emphasis                 |

### Neutral — Gray (text, borders, backgrounds)

| Token       | Usage                            |
| ----------- | -------------------------------- |
| `gray-50`   | Page background, card backgrounds |
| `gray-100`  | Dividers, secondary backgrounds  |
| `gray-200`  | Default borders                  |
| `gray-400`  | Placeholder / muted icons        |
| `gray-500`  | Secondary text, captions         |
| `gray-600`  | Body text (secondary)            |
| `gray-700`  | Body text (primary)              |
| `gray-800`  | Subheadings                      |
| `gray-900`  | **Main headings**, dark text     |

### Links & Info — Blue

| Token       | Usage                   |
| ----------- | ----------------------- |
| `blue-500`  | Inline links            |
| `blue-600`  | Link hover state        |
| `blue-700`  | Strong link emphasis    |

### Feedback — Amber

| Token        | Usage                            |
| ------------ | -------------------------------- |
| `amber-50`   | Gradient destinations            |
| `amber-200`  | Warning card borders             |
| `amber-700`  | Warning icon / badge text        |
| `amber-900`  | Strong warning text              |

### Special

- **Code highlight (polygon)**: `bg-[#0f0]/70 text-gray-900` with `clip-path` hexagon shape
- **Code highlight (rectangle)**: `border-4 border-[#0f0] text-gray-900`

---

## Typography

### Font sizes & weights

| Context              | Size       | Weight                  |
| -------------------- | ---------- | ----------------------- |
| Hero heading (mobile)| `text-4xl` / `text-[2.5rem]` | `font-extrabold` |
| Hero heading (sm+)   | `sm:text-[3.8rem]/16` | `font-extrabold` |
| Section heading      | `text-2xl` / `sm:text-3xl` | `font-bold`     |
| Card heading         | `text-xl`  | `font-semibold`         |
| Body                 | `text-base`| `font-[350]` (light-ish)|
| Caption / label      | `text-sm`  | `font-medium`           |
| Badge / tag          | `text-xs`  | `font-medium uppercase tracking-wide` |

### Letter spacing

- Tight headings: `tracking-tight` (-0.025em)
- Badge / label: `tracking-wide` (0.025em)
- Uppercase tags: `tracking-widest` (0.1em)

### Links in article content

```html
<a class="text-blue-500 underline decoration-dotted hover:opacity-80">
```

---

## Border Radius

| Class        | Value   | Usage                             |
| ------------ | ------- | --------------------------------- |
| `rounded-sm` | 0.25rem | Small elements (badges, tags)     |
| `rounded-md` | 0.375rem| Input fields                      |
| `rounded-lg` | 0.5rem  | Cards, buttons                    |
| `rounded-xl` | 0.75rem | Feature cards, modal panels       |
| `rounded-2xl`| 1rem    | Hero cards, pricing panels        |
| `rounded-full`| 9999px | Pills, avatar images, icon circles|

---

## Shadows

| Class        | Usage                          |
| ------------ | ------------------------------ |
| `shadow-sm`  | Subtle card lift               |
| `shadow`     | Default card                   |
| `shadow-md`  | Hover state elevation          |
| `shadow-lg`  | Floating panels                |
| `shadow-xl`  | Modals, featured pricing card  |

---

## Buttons

### Primary CTA (download / main action)

```html
<a class="inline-flex items-center gap-2 rounded-lg bg-emerald-600 px-5 py-3 text-base font-semibold text-white hover:bg-emerald-700 transition-colors">
  Button Text
</a>
```

### Secondary CTA (pricing)

```html
<a class="inline-flex items-center gap-2 rounded-lg bg-orange-500 px-5 py-3 text-base font-semibold text-white hover:bg-orange-600 transition-colors">
  Buy Now
</a>
```

### Ghost / outline

```html
<a class="inline-flex items-center gap-2 rounded-lg border border-gray-300 px-5 py-3 text-base font-medium text-gray-700 hover:border-gray-400 hover:text-gray-900 transition-colors">
  Learn more
</a>
```

---

## Section Layout

### Standard section

```html
<section class="py-12 sm:py-16">
  <div class="mx-auto max-w-[1280px] px-4 sm:px-8">
    <!-- content -->
  </div>
</section>
```

### Tinted section (feature highlight)

```html
<section class="bg-gray-50 py-12 sm:py-16">
```

### Section heading pattern

```html
<div class="mb-8 text-center">
  <h2 class="text-2xl font-bold tracking-tight text-gray-900 sm:text-3xl">
    Section Title
  </h2>
  <p class="mt-2 text-base text-gray-500">
    Supporting description.
  </p>
</div>
```

---

## Cards

### Feature card (icon + title + body)

```html
<div class="rounded-xl border border-gray-200 bg-white p-6 shadow-sm">
  <div class="mb-3 size-10 rounded-lg bg-emerald-100 flex items-center justify-center">
    <!-- Phosphor icon, text-emerald-600, size-5 -->
  </div>
  <h3 class="text-base font-semibold text-gray-900">Feature name</h3>
  <p class="mt-1 text-sm text-gray-600">Description text.</p>
</div>
```

### Comparison / pricing card

```html
<div class="rounded-2xl border border-gray-200 bg-white p-6 sm:p-8 shadow">
```

---

## Badges & Tags

```html
<!-- Green badge (feature available) -->
<span class="inline-flex items-center gap-1 rounded-full bg-emerald-100 px-2.5 py-0.5 text-xs font-medium text-emerald-700">
  Included
</span>

<!-- Orange badge (pricing / promo) -->
<span class="inline-flex items-center rounded-full bg-orange-100 px-2.5 py-0.5 text-xs font-medium text-orange-700">
  One-time
</span>

<!-- Gray badge (neutral) -->
<span class="inline-flex items-center rounded-full bg-gray-100 px-2.5 py-0.5 text-xs font-medium text-gray-600">
  Open source
</span>
```

---

## Responsive Breakpoints

| Prefix | Min-width | Notes                        |
| ------ | --------- | ---------------------------- |
| (none) | 0px       | Mobile-first base            |
| `sm:`  | 640px     | Small tablet                 |
| `md:`  | 768px     | Tablet / small desktop       |
| `lg:`  | 1024px    | Desktop                      |
| `xl:`  | 1280px    | Wide desktop                 |

---

## Common Grid Patterns

```html
<!-- 3-col feature grid -->
<div class="grid grid-cols-1 gap-4 sm:grid-cols-2 md:grid-cols-3">

<!-- 2-col split (text + image) -->
<div class="flex flex-col gap-8 md:flex-row md:items-center">
  <div class="md:w-3/5"> <!-- text --> </div>
  <div class="md:w-2/5"> <!-- image --> </div>
</div>
```

---

## Dos and Don'ts

**Do:**
- Use `emerald-600` for the primary CTA — it's the brand color
- Use `Inter` font (already loaded via CDN in the base layout)
- Use Phosphor Icons — `ph-bold`, `ph-fill`, or `ph-duotone` classes
- Use `text-gray-900` for headings, `text-gray-700` for body, `text-gray-500` for muted
- Keep section padding consistent: `py-12 sm:py-16`
- Use `transition-colors` on interactive elements

**Don't:**
- Introduce new font families
- Use custom color hex values outside of the ones listed above
- Use dark backgrounds (site is light-mode only, no dark theme)
- Use Heroicons or other icon sets — Phosphor only
- Add `!important` overrides to Tailwind classes
