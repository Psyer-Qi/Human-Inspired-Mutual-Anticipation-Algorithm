import argparse

from pipelines.slm import SLM
from pipelines.rvo import RVO
from pipelines.companion import COMPANION

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="命令行参数解析示例")
    parser.add_argument('--method', type=int, default=None, help='Method index of simulation')
    parser.add_argument('--batch', type=int, default=None, help='Batch index of simulation')
    parser.add_argument('--type', type=int, default=None, help='With/Without ToM')

    args = parser.parse_args()
    print(args)

    methods = [SLM, RVO, COMPANION]

    simulator = methods[args.method](batch=args.batch, with_tom=True if args.type == 1 else False)
    simulator.simulate()
