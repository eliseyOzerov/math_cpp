// Writes the golden values the C++ tests check against, by running the TypeScript the library is ported from.
//
//   node tests/golden/gen.ts            every file
//   node tests/golden/gen.ts fit        one file
//
// node ≥ 23.6 runs the TS as it is. Each cases/<file>.ts imports fountain-ts/src/curves/<file>.ts from its place
// beside this repo (~/Developer/apps/Fountain) and records calls to it; this writes them to <file>.json. Case modules
// import the TS statically: geometry.ts exports `then`, which makes its module namespace a thenable that
// `await import()` never settles.
//
// Each file is a list of cases {fn, args, out}: the function's TS name, its arguments as the TS took them (points as
// [x, y], pieces as lists of 2 or 4 points, option objects as objects) and what it returned. Non-finite numbers are
// the strings "NaN", "Infinity" and "-Infinity"; undefined is null. The JSON is committed, so the tests need no node.
import { readdirSync, writeFileSync } from 'node:fs'
import { dirname, join } from 'node:path'
import { fileURLToPath } from 'node:url'
import type { Case } from './cases/record.ts'

const here = dirname(fileURLToPath(import.meta.url))
const shared = new Set(['record', 'fixtures'])
const only = process.argv.slice(2)
const files = readdirSync(join(here, 'cases')).filter(f => f.endsWith('.ts')).map(f => f.slice(0, -3))
  .filter(f => !shared.has(f) && (!only.length || only.includes(f)))

const json = (v: unknown) => JSON.stringify(v, (_, x) =>
  x === undefined ? null : typeof x === 'number' && !Number.isFinite(x) ? String(x) : x)
for (const file of files) {
  const { default: cases } = await import(`./cases/${file}.ts`) as { default: () => Case[] }
  const list = cases()
  writeFileSync(join(here, `${file}.json`), '[\n' + list.map(json).join(',\n') + '\n]\n')
  console.log(`${file}.json: ${list.length} cases`)
}
