import type { Simulation } from '@/engine/simulation';

const CRASH = 'bg-[#e66767]/90 text-black';
const LANDED = 'bg-[#5bbf8a]/90 text-black';

/** How the flight ended, in the words of whatever flies, or null while it goes on. */
export function endNotice(s: Simulation): [string, string] | null {
  const reset = 'Press R to reset.';
  if (s.rocket?.landed || s.pdgLander?.landed) {
    const v = s.rocket ? Math.abs(s.rocket.touchdown) : s.pdgLander!.touchdownSpeed;
    return [`Landed at ${v.toFixed(2)} m/s.`, LANDED];
  }
  if (!s.state.crashed) return null;
  if (s.rocket)
    return [
      `Crashed! Touched down at ${Math.abs(s.rocket.touchdown).toFixed(1)} m/s. ${reset}`,
      CRASH,
    ];
  if (s.pdgLander)
    return [
      `Crashed! Touched down at ${s.pdgLander.touchdownSpeed.toFixed(1)} m/s. ${reset}`,
      CRASH,
    ];
  if (s.tvc)
    return [`Broke up: the rocket turned past 30° and the air tore it apart. ${reset}`, CRASH];
  if (s.aircraft) return [`Crashed! The aircraft flew into the ground. ${reset}`, CRASH];
  return [`Crashed! Hit the ground too hard. ${reset}`, CRASH];
}
