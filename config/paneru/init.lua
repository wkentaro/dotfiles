paneru.setup {
    options = {
        focus_follows_mouse = false,
        mouse_follows_focus = false,
        animation_speed = 15.0,
        preset_column_widths = { 0.33333, 0.5, 0.66667, 1.0 },
        -- Keep numbered workspaces stable when their last window closes.
        reap_empty_workspaces = false,
    },
    padding = { top = 10, bottom = 10, left = 10, right = 10 },
    swipe = { scroll = { modifier = "alt", vertical_modifier = "shift" } },
    bindings = {
        ["window focus west"] = "alt - leftarrow",
        ["window focus east"] = "alt - rightarrow",
        ["window swap west"] = "alt + shift - leftarrow",
        ["window swap east"] = "alt + shift - rightarrow",
        ["window manage"] = "alt - t",
        ["window fullwidth"] = "alt - f",
        ["window center"] = "alt + shift - 0",
        ["window grow"] = "alt + shift - equal",
        ["window shrink"] = "alt + shift - minus",
        ["window virtualnum 1"] = "alt - 1",
        ["window virtualnum 2"] = "alt - 2",
        ["window virtualnum 3"] = "alt - 3",
        ["window virtualnum 4"] = "alt - 4",
        ["window virtualnum 5"] = "alt - 5",
        ["window virtualmovenum 1"] = "alt + shift - 1",
        ["window virtualmovenum 2"] = "alt + shift - 2",
        ["window virtualmovenum 3"] = "alt + shift - 3",
        ["window virtualmovenum 4"] = "alt + shift - 4",
        ["window virtualmovenum 5"] = "alt + shift - 5",
        quit = "ctrl + alt - q",
    },
    windows = {
        all = { title = ".*", horizontal_padding = 5, vertical_padding = 5 },
        labelme = { title = "^Labelme($| - )", floating = true },
        settings = { title = ".*", bundle_id = "com.apple.systempreferences", floating = true },
    },
}

dofile(os.getenv("HOME") .. "/.config/paneru/float-workspace.lua")

paneru.bind("cmd + ctrl - grave", function(ws)
    local focused = ws:focused()
    local display = focused and ws:display_of(focused)
    if not display then return end
    for _, window in ipairs(ws:windows()) do
        local target = ws:display_of(window.id)
        if window.visible and target and target.id ~= display.id then
            return ws:focus(window.id)
        end
    end
end)
