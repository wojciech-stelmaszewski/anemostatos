# anemostatos

An educational, interactive WebGL simulator for understanding the **PID
controller**: a quadcopter hovers above a reference grid, fighting gravity and
random wind gusts, while every term of its controller is plotted live and
every parameter can be tweaked.

Three levels of fidelity — a single altitude loop, a 3D point mass, and a full
quadrotor with a cascade of controllers — plus 14 guided lessons.

## Run it

Requires Node.js ≥ 20.

```sh
make dev      # install dependencies if needed, start at http://localhost:5173
make check    # lint + typecheck + tests
make          # list all targets
```

## Documentation

See [docs/](docs/README.md): the PID primer, physics model, control and
software architecture, UI design and roadmap.
