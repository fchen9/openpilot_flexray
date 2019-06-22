from multiprocessing import Process
from bf7 import start, BFAlgo7, load_progress

if __name__ == '__main__':
  while True:
    values = BFAlgo7.generate_all_values()
    progress = load_progress()
    print('Cur progress: {}/{}'.format(progress, len(values)))
    if progress == len(values):
      break
    p = Process(target=start)
    p.start()
    p.join()