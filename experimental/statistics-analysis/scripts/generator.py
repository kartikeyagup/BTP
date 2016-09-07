import os
import itertools

def main():
  num_threads=40
  distances = [i for i in xrange(1,51,10)]
  angles = [i for i in xrange(0,30,10)]
  starting_points_z = [i for i in xrange(300,500,100)]
  num_images=[2]
  dump_images=[0]
  verbose=[0]
  allpossibilites = itertools.product(distances,angles,starting_points_z,num_images,dump_images,verbose)
  for elem in list(allpossibilites):
    simulateData(elem)

def simulateData((distance,angle,starting_point,num_images,dump_images,verbose)):
  dirname = "data/"+"_".join(["d",str(distance),"a",str(angle),"stz",str(starting_point),"n",str(num_images)])
  command = "build/sensitivity"
  command += " --distance="+str(distance)
  command += " --dirname="+dirname
  command += " --angle="+str(angle)
  command += " --starting_z="+str(starting_point)
  command += " --num_images="+str(num_images)
  command += " --dump_images="+str(dump_images)
  command += " --verbose="+str(verbose)    
  os.system(command)

if __name__ == '__main__':
  main()
