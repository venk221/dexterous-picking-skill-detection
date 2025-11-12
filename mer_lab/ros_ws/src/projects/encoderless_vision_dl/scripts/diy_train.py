from pathlib import Path
import argparse 
from arch_models import KpRcnn
from datasets import KpRcnnDataset
import network_train as train

class MyFormatter(argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter):
        pass

parser = argparse.ArgumentParser(
        description="To train from scratch",
        formatter_class=MyFormatter
    )

parser.add_argument("model", type=str)

args = parser.parse_args()

if args.model == 'KpRcnn':
    train.train_kprcnn()
elif args.model == 'Resnet':
    train.train_resnet()
