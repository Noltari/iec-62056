"""IEC 62056 client simulation."""

import argparse
import asyncio
import logging

from . import IEC62506

_LOGGER = logging.getLogger(__name__)


async def iec_62056_client(args: argparse.Namespace) -> None:
    """IEC 62056 client function."""

    iec62056 = IEC62506(args)
    await iec62056.client()


async def main():
    """IEC 62056 client entry."""
    parser = argparse.ArgumentParser(
        prog="IEC 62056",
        description="IEC 62056 client simulation",
    )
    IEC62506.cls_parser_args(parser)
    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)

    await iec_62056_client(args)


if __name__ == "__main__":
    asyncio.run(main())
