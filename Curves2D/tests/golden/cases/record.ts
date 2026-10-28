// How a case module records its cases: `export default record(module, F => { F('name', ...args) })` runs each named
// function of the TS module on the arguments and keeps {fn, args, out}.
export type Case = { fn: string, args: unknown[], out: unknown }
export type F = (fn: string, ...args: unknown[]) => void

export function record(mod: object, cases: (F: F) => void): () => Case[] {
  return () => {
    const list: Case[] = []
    cases((fn, ...args) => {
      const f = (mod as Record<string, unknown>)[fn]
      if (typeof f !== 'function') throw new Error(`no ${fn}`)
      const before = structuredClone(args)
      list.push({ fn, args: before, out: (f as (...a: unknown[]) => unknown)(...args) })
    })
    return list
  }
}
