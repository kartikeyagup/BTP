import subprocess

def main():
  for i in xrange(1,101,10):
    print "Calling", i
    dirname = "data/distance_" + str(i)
    command = "build/sensitivity --distance="+str(i)+" --dirname="+dirname
    # print command
    # continue
    subprocess.call(command, shell=True)

if __name__ == '__main__':
  main()
