-- WezTerm configured to look and feel like Kaku (a WezTerm fork with clean
-- macOS defaults). The point is one terminal that looks the same on macOS and
-- Linux; Kaku is macOS-only, so its bundled defaults are reproduced here.

local wezterm = require 'wezterm'

local config = wezterm.config_builder and wezterm.config_builder() or {}

local is_macos = wezterm.target_triple:find 'darwin' ~= nil

-- ===== Color scheme =====
-- Matches the Catppuccin Mocha used in ghostty and the personal kaku.lua.
-- Swap this single line for any built-in scheme (e.g. 'Kaku Dark' is not
-- bundled, but 'Tokyo Night', 'Dracula', etc. are) to retheme everything.
config.color_scheme = 'Catppuccin Mocha'

-- Catppuccin Mocha tokens reused for the titlebar so the integrated traffic
-- lights blend into the terminal background like Kaku's.
local BASE = '#1e1e2e'
local SURFACE = '#313244'
local TEXT = '#cdd6f4'
local OVERLAY = '#6c7086'
local FONT_FAMILY = 'JetBrains Mono'

-- ===== Font =====
-- No font_rules on purpose: Kaku's bundled rules flatten bold/italic/faint
-- (faint forced to base weight), which kills dimming on e.g. Claude Code's
-- suggestion text. Relying on the full JetBrains Mono family lets WezTerm
-- derive real bold/italic and dim faint text natively. Emoji/CJK fallbacks are
-- per-OS to avoid missing-font warnings on Linux.
local font_fallback = { { family = FONT_FAMILY } }
if is_macos then
  table.insert(font_fallback, { family = 'PingFang SC' })
  table.insert(font_fallback, 'Apple Color Emoji')
else
  table.insert(font_fallback, 'Noto Color Emoji')
end

config.font = wezterm.font_with_fallback(font_fallback)
config.font_size = is_macos and 13.0 or 9.0
-- line_height >= ~1.28 trips a WezTerm glyph-rendering bug on Linux (renders a
-- serif fallback instead of JetBrains Mono); 1.25 stays just below that cliff.
config.line_height = 1.25
config.harfbuzz_features = { 'calt=0', 'clig=0', 'liga=0' }
config.bold_brightens_ansi_colors = false
config.use_cap_height_to_scale_fallback_fonts = false
config.custom_block_glyphs = true
config.unicode_version = 14

-- ===== Window =====
config.window_decorations = 'INTEGRATED_BUTTONS|RESIZE'
config.window_padding = { left = '8px', right = '8px', top = '8px', bottom = '0px' }
config.initial_cols = 110
config.initial_rows = 22
config.use_resize_increments = false
config.adjust_window_size_when_changing_font_size = false
config.window_close_confirmation = 'NeverPrompt'

config.window_frame = {
  font = wezterm.font { family = FONT_FAMILY, weight = 'Regular' },
  font_size = is_macos and 14.0 or 11.0,
  active_titlebar_bg = BASE,
  inactive_titlebar_bg = BASE,
  active_titlebar_fg = TEXT,
  inactive_titlebar_fg = OVERLAY,
}

-- ===== Tab bar =====
-- Retro (non-fancy) tab bar, hidden entirely with a single tab; this is why
-- Kaku shows no tab strip most of the time.
config.tab_bar_at_bottom = false
config.use_fancy_tab_bar = false
config.tab_max_width = 32
config.hide_tab_bar_if_only_one_tab = true
config.show_tab_index_in_tab_bar = true
config.show_new_tab_button_in_tab_bar = false
config.colors = {
  tab_bar = {
    background = BASE,
    active_tab = { bg_color = SURFACE, fg_color = TEXT, intensity = 'Bold' },
    inactive_tab = { bg_color = BASE, fg_color = OVERLAY },
    inactive_tab_hover = { bg_color = SURFACE, fg_color = TEXT },
    new_tab = { bg_color = BASE, fg_color = OVERLAY },
    new_tab_hover = { bg_color = SURFACE, fg_color = TEXT },
  },
}

-- ===== Cursor =====
config.default_cursor_style = 'BlinkingBar'
config.cursor_thickness = '2px'
config.cursor_blink_rate = 500
config.cursor_blink_ease_in = 'Constant'
config.cursor_blink_ease_out = 'Constant'

-- ===== Scrollback / selection =====
config.scrollback_lines = 10000
config.selection_word_boundary = ' \t\n{}[]()"\'-'

-- ===== Keyboard =====
-- Negotiate the Kitty keyboard protocol so modified chords (alt+shift+k/j for
-- herdr workspace nav) arrive unambiguously, then send raw bytes for the chords
-- herdr expects instead of the protocol's CSI-u encodings. Mirrors kaku.lua.
config.enable_kitty_keyboard = true
config.keys = {
  { key = 'Enter', mods = 'CTRL', action = wezterm.action.SendString '\r' },
  { key = 'Enter', mods = 'SHIFT', action = wezterm.action.SendString '\n' },
  { key = '[', mods = 'CTRL', action = wezterm.action.SendString '\x1b' },
  -- macOS treats Option+N as a dead key; send legacy meta-n so herdr's
  -- next_tab (alt+n) fires. Harmless on Linux where alt+n already maps here.
  { key = 'n', mods = 'ALT', action = wezterm.action.SendString '\x1bn' },
  -- Close tab without the "Really kill this tab?" prompt (herdr manages its
  -- own panes). Covers the macOS and Linux default close-tab shortcuts.
  { key = 'w', mods = 'CMD', action = wezterm.action.CloseCurrentTab { confirm = false } },
  { key = 'w', mods = 'CTRL|SHIFT', action = wezterm.action.CloseCurrentTab { confirm = false } },
}

return config
