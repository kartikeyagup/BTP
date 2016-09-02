import matplotlib as plt
import numpy as np

def main():
  '''
  f = open('points.txt', 'r')
  for line in f:
    #plot stuff
  '''

  t = np.arange(0.0, 2.0, 0.01)
  s = np.sin(2*np.pi*t)
  plt.plot(t, s)

  plt.xlabel('time (s)')
  plt.ylabel('voltage (mV)')
  plt.title('About as simple as it gets, folks')
  plt.grid(True)
  plt.savefig("test.png")
  plt.show()  


if __name__ == '__main__':
  main()
