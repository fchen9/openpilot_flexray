from multiprocessing import Process
import time
import sys
from bf5 import start, BFAlgo5, load_progress

if __name__ == '__main__':
  while True:
    values = BFAlgo5.generate_all_values()
    progress = load_progress()
    print('Cur progress: {}/{}'.format(progress, len(values)))
    if progress == len(values):
      break
    p = Process(target=start)
    p.start()
    p.join()