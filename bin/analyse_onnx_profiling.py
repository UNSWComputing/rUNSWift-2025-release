import json
import argparse
from collections import defaultdict

def analyze_onnx_profile(profile_path):
    with open(profile_path, 'r') as f:
        profile_data = json.load(f)

    # Analyze operator execution times
    op_times = defaultdict(float)
    op_calls = defaultdict(int)
    total_time = 0

    for event in profile_data:
        if 'args' in event and 'op_name' in event['args']:
            op_name = event['args']['op_name']
            duration = event.get('dur', 0)  # Duration in microseconds
            
            op_times[op_name] += duration
            op_calls[op_name] += 1
            total_time += duration

    # Sort operators by total time
    sorted_ops = sorted(op_times.items(), key=lambda x: x[1], reverse=True)

    print("\n=== ONNX Runtime Profile Analysis ===\n")
    print(f"Total execution time: {total_time/1000:.2f} ms")
    print("\nTop operators by execution time:")
    print("-" * 60)
    print(f"{'Operator':<30} {'Time (ms)':<12} {'Calls':<8} {'Avg (ms)':<12}")
    print("-" * 60)

    for op_name, time in sorted_ops:
        time_ms = time / 1000  # Convert to milliseconds
        calls = op_calls[op_name]
        avg_time = time_ms / calls if calls > 0 else 0
        print(f"{op_name[:30]:<30} {time_ms:>10.2f}ms {calls:>8} {avg_time:>10.2f}ms")

    # Thread utilization analysis
    if 'ph' in profile_data[0]:  # Chrome trace format
        thread_times = defaultdict(float)
        for event in profile_data:
            if event.get('ph') == 'X':  # Complete events
                thread_id = event.get('tid', 'unknown')
                thread_times[thread_id] += event.get('dur', 0)

        print("\nThread utilization:")
        print("-" * 40)
        for thread_id, time in thread_times.items():
            print(f"Thread {thread_id}: {time/1000:.2f}ms")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Analyze ONNX Runtime profile data')
    parser.add_argument('profile_file', help='Path to the ONNX Runtime profile JSON file')
    args = parser.parse_args()

    analyze_onnx_profile(args.profile_file)