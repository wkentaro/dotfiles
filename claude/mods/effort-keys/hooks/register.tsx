import type { EngineInterface, Register } from 'claude-code'

const LEVELS = ['low', 'medium', 'high', 'xhigh', 'max']

// The session's level as last seen: a request's own, or the one set here.
let current: string | undefined

const saved = async ($: EngineInterface) => {
  const model = (await $.session.model()).replace(/\[.*\]$/, '')
  const { modelSettings } = (await $.settings.read()) as {
    modelSettings?: Record<string, { effortLevel?: string }>
  }
  // ponytail: a model with no saved level is taken as medium; its real default
  // may differ until the first request shows it.
  const level = modelSettings?.[model]?.effortLevel
  return level !== undefined && LEVELS.includes(level) ? level : 'medium'
}

const bump = async ($: EngineInterface, step: number) => {
  const i = LEVELS.indexOf(current ?? (await saved($))) + step
  const to = LEVELS[Math.max(0, Math.min(LEVELS.length - 1, i))]
  if (to === undefined || to === current) return
  current = to
  // The real /effort, so the engine's own indicator and requests follow.
  await $.command.run({ command: 'effort', args: to })
}

export const register: Register = on => {
  // The Buttons borrow the diff panel's file-list actions, which the engine
  // handles only while that panel is open, so their keys press these from the
  // prompt: meta+up/down (and ctrl+up/down) by default. A bare shift+arrow never presses one.
  on('ui.render', { component: 'AbovePrompt' }, ($, e, next) => {
    if (e.props.hasSurvey) return next(e)
    const { Box, Button } = $.ui.resolve(e)

    // Hidden: the Buttons only need to be mounted for their keys to press them.
    return (
      <Box display="none">
        <Button label="up" action="app:diffFileListUp" onPress={() => bump($, 1)} />
        <Button label="down" action="app:diffFileListDown" onPress={() => bump($, -1)} />
      </Box>
    )
  })

  on('turn.step', async function* ($, e, next) {
    if (e.agentId === undefined && typeof e.effort === 'string') current = e.effort
    return yield* next(e)
  })
}
