import argparse
from orca_core import OrcaHand

def main():
    parser = argparse.ArgumentParser(
        description="Calibrate the ORCA Hand. Specify the path to the orcahand model folder."
    )
    parser.add_argument(
        "model_path",
        type=str,
        nargs="?",
        default=None,
        help="Path to the orcahand model folder (e.g., /path/to/orcahand_v1)"
    )
    args = parser.parse_args()

    hand = OrcaHand(args.model_path)
    status = hand.connect()
    print(status)

    if not status[0]:
        print("Failed to connect to the hand.")
        exit(1)

    hand.calibrate()

if __name__ == "__main__":
    main()