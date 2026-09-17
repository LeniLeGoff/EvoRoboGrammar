import pandas as pd
import sys

import matplotlib.pyplot as plt

def plot_fitness(csv_file):
    """Plot fitness values from a CSV file as a scatter plot."""
    df = pd.read_csv(csv_file)
    df.columns = ['id', 'parent1_id', 'parent2_id', 'fitness']
    plt.figure(figsize=(10, 6))
    plt.scatter(df['id'], df['fitness'], alpha=0.6, edgecolors='k')
    plt.xlabel('ID')
    plt.ylabel('Fitness')
    plt.title('Fitness Values')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python plot_fitness.py <csv_file>")
        sys.exit(1)
    
    plot_fitness(sys.argv[1])