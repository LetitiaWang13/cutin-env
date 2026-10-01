"""Draw one scenario, without retrying or running physics."""

import argparse
import json
import tomllib
from pathlib import Path
from random import Random

from cutin.application import load_scenario
from cutin.generation import RejectedSample, SearchSpace, sample


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "base", nargs="?", type=Path, default=Path("examples/baseline.toml")
    )
    parser.add_argument(
        "--space", type=Path, default=Path("examples/search_space.toml")
    )
    parser.add_argument(
        "--seed", type=int, default=17, help="Proposal RNG seed, not scenario seed"
    )
    parser.add_argument(
        "--output", type=Path, help="New concrete scenario JSON; never overwrite"
    )
    args = parser.parse_args()
    try:
        if args.output and (args.output.exists() or args.output.is_symlink()):
            raise FileExistsError(f"output already exists: {args.output}")
        base = load_scenario(args.base)
        space = SearchSpace.from_dict(
            tomllib.loads(args.space.read_text(encoding="utf-8"))
        )
        candidate = sample(base, space, Random(args.seed))
        text = json.dumps(candidate.to_dict(), indent=2, allow_nan=False) + "\n"
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("x", encoding="utf-8") as output:
                output.write(text)
            print("Saved:", args.output)
        else:
            print(text, end="")
    except RejectedSample as error:
        parser.exit(1, f"Rejected sample: {error}\n")
    except (OSError, ValueError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
