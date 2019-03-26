import psutil
import time
import os
import sys
import numpy as np
import argparse
import re

# Do statistics every 5 seconds
PRINT_INTERVAL = 5
SLEEP_INTERVAL = 0.2

def get_arg_parser():
  parser = argparse.ArgumentParser(
    description="Unlogger and UI",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)

  parser.add_argument("proc_names", nargs="?",
                      help="Process names to be monitored, comma seperated")
  parser.add_argument("--list_all", nargs="?", type=bool, default=False,
                      help="Show all running processes' cmdline")
  return parser


if __name__ == "__main__":
  args = get_arg_parser().parse_args(sys.argv[1:])
  if args.list_all:
    for p in psutil.process_iter():
      print('cmdline', p.cmdline(), 'name', p.name())
    sys.exit(0)

  proc_names = args.proc_names.split(',')
  monitored_procs = []
  stats = {}
  for p in psutil.process_iter():
    if p == psutil.Process():
      continue
    matched = any([l for l in p.cmdline() if any([pn for pn in proc_names if re.match(r'.*{}.*'.format(pn), l, re.M | re.I)])])
    if matched:
      k = ' '.join(p.cmdline())
      print('Add monitored proc:', k)
      stats[k] = {'cpu_percents': [], 'acc_avg_cpu': None}
      monitored_procs.append(p)
  i = 0
  interval_int = int(PRINT_INTERVAL / SLEEP_INTERVAL)
  while True:
    for p in monitored_procs:
      k = ' '.join(p.cmdline())
      stats[k]['cpu_percents'].append(p.cpu_percent())
      if len(stats[k]['cpu_percents']) > 25:
        stats[k]['cpu_percents'] = stats[k]['cpu_percents'][1:]
    time.sleep(SLEEP_INTERVAL)
    i += 1
    if i % interval_int == 0:
      for k, stat in stats.items():
        if len(stat['cpu_percents']) <= 0:
          continue
        avg_cpu = np.array(stat['cpu_percents']).mean()
        stat['cpu_percents'] = []
        if not stat['acc_avg_cpu']:
          stat['acc_avg_cpu'] = avg_cpu
        else:
          stat['acc_avg_cpu'] = (stat['acc_avg_cpu'] + avg_cpu) / 2
        print(os.path.basename(k), 'cur', '{0:.2f}'.format(avg_cpu), 'acc', '{0:.2f}'.format(stat['acc_avg_cpu']))
