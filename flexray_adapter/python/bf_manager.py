from multiprocessing import Process
import time
import sys
from bf6 import start, BFAlgo6, load_progress

if __name__ == '__main__':
  while True:
    values = BFAlgo6.generate_all_values()
    progress = load_progress()
    print('Cur progress: {}/{}'.format(progress, len(values)))
    if progress == len(values):
      break
    p = Process(target=start)
    p.start()
    p.join()