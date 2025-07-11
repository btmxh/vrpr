import pandas as pd
import matplotlib.pyplot as plt
import re
import glob

filepaths = glob.glob("results-g50-d4-k4-cr*.csv")

avg_fitness = {}

for path in filepaths:
    match = re.search(r"cr(\d+)", path)
    if not match:
        continue
    # Convert e.g. "005" → 0.005
    cr_val = float("0." + match.group(1)) * 10

    df = pd.read_csv(path)
    avg_fitness[cr_val] = df["fitness"].mean()

# Sort by numeric cr
sorted_items = sorted(avg_fitness.items())
labels = [str(cr) for cr, _ in sorted_items]
values = [fitness for _, fitness in sorted_items]

plt.figure(figsize=(8, 6))
plt.bar(labels, values, color="lightcoral")
plt.xlabel("cr Value")
plt.ylabel("Average Fitness")
plt.title("Average Fitness for Different cr Values (k=4)")
plt.grid(axis="y")
plt.tight_layout()
plt.savefig("avg_fitness_cr_values.png")
plt.show()
