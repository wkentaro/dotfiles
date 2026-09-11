local function plan_float_changes(windows, owned, enable)
    local changes = {}
    local remembered = {}
    for id, bundle in pairs(owned or {}) do remembered[id] = bundle end
    for _, window in ipairs(windows) do
        local id = tostring(window.window_id)
        if enable and not window.floating then
            remembered[id] = window.bundle_id
            changes[#changes + 1] = window.window_id
        elseif not enable and remembered[id] == window.bundle_id then
            changes[#changes + 1] = window.window_id
        end
    end
    return changes, remembered
end

if ... == "--check" then
    local windows = {
        { window_id = 1, bundle_id = "terminal", floating = false },
        { window_id = 2, bundle_id = "settings", floating = true },
    }
    local changes, owned = plan_float_changes(windows, nil, true)
    assert(#changes == 1 and changes[1] == 1 and owned["2"] == nil)
    windows[1].floating = true
    windows[3] = { window_id = 3, bundle_id = "browser", floating = false }
    changes, owned = plan_float_changes(windows, owned, true)
    assert(#changes == 1 and changes[1] == 3 and owned["1"] == "terminal")
    windows[3].floating = true
    changes = plan_float_changes(windows, owned, false)
    assert(#changes == 2 and changes[1] == 1 and changes[2] == 3)
    windows[1].bundle_id = "different-app"
    changes = plan_float_changes(windows, owned, false)
    assert(#changes == 1 and changes[1] == 3)
    assert(#plan_float_changes({}, owned, false) == 0)
    print("Workspace floating checks passed")
    return
end

local function find_current_workspace()
    local state = paneru.query_state()
    for _, workspace in ipairs(state.virtual_workspaces) do
        if workspace.native_workspace_id == state.active.native_workspace_id
            and workspace.number == state.active.virtual_workspace_number then
            return workspace
        end
    end
end

local function apply_float_mode(ws, toggle)
    local workspace = find_current_workspace()
    if not workspace then return end
    -- Virtual workspace numbers repeat across displays and native Spaces.
    local key = "floating:" .. workspace.native_workspace_id .. ":" .. workspace.number
    local changes, enable
    -- Spawn events and keybindings can overlap while awaiting the state store.
    paneru.state.mutate(key, function(mode)
        if not toggle and not mode then
            changes = nil
            return nil
        end
        enable = not toggle or not mode
        local owned
        changes, owned = plan_float_changes(workspace.windows, mode and mode.owned, enable)
        return enable and { owned = owned } or nil
    end)
    if not changes then return end
    for _, id in ipairs(changes) do
        ws = enable and ws:float(id) or ws:sink(id)
    end
    if toggle then paneru.flash(enable and "Workspace: floating" or "Workspace: tiling") end
    return ws
end

paneru.bind("alt + shift - t", function(ws)
    return apply_float_mode(ws, true)
end)
paneru.on("window_spawned", function(_, ws) return apply_float_mode(ws, false) end)
paneru.on("space_changed", function(_, ws) return apply_float_mode(ws, false) end)
