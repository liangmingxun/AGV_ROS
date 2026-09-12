#!/usr/bin/env python3
"""Read-only diagnostics for bags recorded after the state-chain repair."""
import argparse
import collections
import rosbag

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('bag')
args = parser.parse_args()
fusion = collections.defaultdict(dict)
stages = collections.defaultdict(lambda: collections.defaultdict(list))
suppressed = collections.Counter()
estimator_receipts = []
topics = ['/pose_provider/agv%d/%s' % (n, suffix)
          for n in (1, 2, 3) for suffix in ('fusion_timing', 'estimator_timing')]
with rosbag.Bag(args.bag) as bag:
    for topic, msg, stamp in bag.read_messages(topics=topics):
        robot = topic.split('/')[2]
        a = msg.data
        if topic.endswith('/fusion_timing') and len(a) == 7:
            source, receipt, callback, publish = a[1:5]
            stages[robot]['odom_source_to_receipt'].append(receipt-source)
            stages[robot]['fusion_callback_queue_wait'].append(callback-receipt)
            stages[robot]['fusion_callback_processing'].append(publish-callback)
            fusion[robot][source] = publish
            suppressed[robot] += a[5] == 0
        elif topic.endswith('/estimator_timing') and len(a) == 5:
            stages[robot]['estimator_source_age'].append(a[3])
            estimator_receipts.append((robot, a[1], a[2]))
for robot, source, callback in estimator_receipts:
    if source in fusion[robot]:
        stages[robot]['fusion_publish_to_estimator_callback'].append(callback-fusion[robot][source])
if not stages:
    raise SystemExit('No state-chain timing topics: use a newly recorded bag.')
for robot in sorted(stages):
    print(robot, 'suppressed_stale_fusion_outputs=', suppressed[robot])
    for name, values in stages[robot].items():
        values.sort()
        print('  %s: n=%d p95=%.3f ms max=%.3f ms' %
              (name, len(values), 1000*values[int(.95*(len(values)-1))], 1000*values[-1]))
