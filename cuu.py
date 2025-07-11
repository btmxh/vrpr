import glob
import json
import csv
from pathlib import Path
from sys import argv

prefix = argv[1] if len(argv) > 1 else ""

# Expand path for your files
files = glob.glob(
    str(
        Path(
            f"~/Downloads/Instance/WithTimeWindows/{prefix}*.json.result.jsonc"
        ).expanduser()
    )
)

# Initialize a list to store CSV rows
rows = []

# Iterate over each file
for path in files:
    with open(path) as f:
        last_full_result = None
        for line in f:
            try:
                obj = json.loads(line)
                if obj.get("_") == "full_result":
                    last_full_result = obj
            except json.JSONDecodeError:
                continue  # skip malformed lines

        if last_full_result:
            filename = Path(path).name

            # Extracting additional fields from the corresponding static JSON file
            static_file_path = (
                str(path)
                .replace("WithTimeWindows", "Static_solution_noTimeWindow")
                .replace(".json.result.jsonc", ".json")
            )
            with open(static_file_path) as static_f:
                static_obj = json.load(static_f)
                elapsed_time = static_obj.get("elapsed")
                truck_working_time = static_obj.get("solution", {}).get(
                    "truck_working_time", [None]
                )[0]
                drone_working_time = static_obj.get("solution", {}).get(
                    "drone_working_time", [None]
                )[0]

            # Calculate the minimum working time
            min_working_time = (
                max(truck_working_time, drone_working_time)
                if truck_working_time is not None and drone_working_time is not None
                else None
            )

            # Appending to rows
            rows.append(
                {
                    "filename": filename,
                    "fitness": last_full_result.get("fitness"),
                    "makespan": last_full_result.get("result", [None])[0],
                    "missed": last_full_result.get("result", [None, None])[1],
                    "baseline": min_working_time,
                }
            )

# Write the data to a CSV file
with open("results.csv", "w", newline="") as csvfile:
    writer = csv.DictWriter(
        csvfile,
        fieldnames=[
            "filename",
            "fitness",
            "makespan",
            "missed",
            "baseline",
        ],
    )
    writer.writeheader()
    writer.writerows(rows)

print("✅ Done. Wrote to results.csv")
