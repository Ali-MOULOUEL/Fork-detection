import argparse

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=str)
    parser.add_argument("scenarios", type=str)
    return parser.parse_args()

if __name__== "__main__" :
    args = parse_args()
    print(args.bag)
    print(args.scenarios)
