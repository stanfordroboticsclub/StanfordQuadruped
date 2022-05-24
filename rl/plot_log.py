import matplotlib.pyplot as plt
import argparse
import pandas as pd

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--log',
                        required=True,
                        type=str,
                        help="Path to log file")
    args = parser.parse_args()

    df = pd.read_csv(args.log, delimiter='\t')
    print(df.columns)
    plt.plot(df['Iteration'], df['AverageReward'])
    plt.fill_between(df['Iteration'],
                     df['MinRewardRollout'],
                     df['MaxRewardRollout'],
                     alpha=0.2)
    plt.legend(["AverageReward","MinReward to MaxReward"])
    plt.ylim(200,202)
    plt.show()
