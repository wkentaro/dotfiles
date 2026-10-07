import { expect, test } from 'claude-code/testing'

test('the band buttons run /effort one level up or down from the last request', async ($, on) => {
  const ran: string[] = []
  on('command.run', (_, e) => {
    ran.push(`${e.command} ${e.args}`)
    return { text: '' }
  })
  on('turn.step', async function* (_, e) {
    return { turnId: e.turnId, index: e.index, answer: '', toolUses: [] } as never
  })

  for await (const _ of $.turn.step({ turnId: 't', index: 0, model: 'm', effort: 'high', messageCount: 1 })) void _
  const band = await $.ui.mount({ plugin: 'effort-keys', surface: 'terminal', component: 'AbovePrompt', props: {} as never })
  await band.press({ key: 'up' })
  await band.press({ key: 'up' })
  await band.press({ key: 'up' })
  await band.press({ key: 'down' })

  expect(ran).toEqual(['effort xhigh', 'effort max', 'effort xhigh'])
})
