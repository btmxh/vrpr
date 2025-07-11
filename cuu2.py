import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Parameters
file_path = "results-g100-d4-k4.csv"
group_size = 100

# Load data
df = pd.read_csv(file_path)

# Extract group from filename
filename = file_path.split("/")[-1]
depth = int(filename.split("-d")[1].split("-")[0])

# Aggregate by group (e.g., 6.10.1 -> 6)
print(df["filename"])
df["group"] = df["filename"].str.extract(r"^(\d+)").astype(int)
df["miss_rate"] = df["missed"] / df["group"]

avg_makespan = df.groupby("group")["makespan"].mean()
avg_baseline = df.groupby("group")["baseline"].mean()
avg_miss_rate = (
    df.groupby("group")["miss_rate"].sum() / df["group"].value_counts().sort_index()
)

print(avg_miss_rate)

# Prepare data
x = np.arange(len(avg_makespan))
group_labels = avg_makespan.index.tolist()

# Plotting
fig, ax1 = plt.subplots(figsize=(10, 6))
bar_width = 0.35

# Bar chart for makespan and baseline
ax1.bar(
    x - bar_width / 2, avg_makespan, width=bar_width, label="Makespan", color="skyblue"
)
ax1.bar(
    x + bar_width / 2,
    avg_baseline,
    width=bar_width,
    label="Baseline",
    color="lightcoral",
)
ax1.set_xlabel("Group")
ax1.set_ylabel("Makespan / Baseline")
ax1.set_xticks(x)
ax1.set_xticklabels(group_labels)
ax1.legend(loc="upper left")
ax1.grid(True, axis="y")

# Line chart for miss rate
ax2 = ax1.twinx()
ax2.plot(x, avg_miss_rate, color="green", marker="o", label="Miss Rate")
ax2.set_ylabel("Miss Rate")
ax2.legend(loc="upper right")

plt.title(f"Makespan vs Baseline and Miss Rate ({file_path})")
plt.tight_layout()
plt.savefig("makespan_baseline_missrate_groupwise.png")
plt.show()
