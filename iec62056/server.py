"""IEC 62056 server simulation."""

import argparse
import asyncio
import logging

from . import IEC62506

_LOGGER = logging.getLogger(__name__)


async def iec_62056_server(args: argparse.Namespace) -> None:
    """IEC 62056 server function."""

    iec62056 = IEC62506(args)
    await iec62056.server()


async def main():
    """IEC 62056 server entry."""
    parser = argparse.ArgumentParser(
        prog="IEC 62056",
        description="IEC 62056 server simulation",
    )
    IEC62506.cls_parser_args(parser)
    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)

    await iec_62056_server(args)


if __name__ == "__main__":
    asyncio.run(main())
