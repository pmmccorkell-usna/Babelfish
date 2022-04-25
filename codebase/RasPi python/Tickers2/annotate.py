filename = "/dev/shm/mjpeg/user_annotate.txt"
while True:
  ann = raw_input("Enter annotation (empty to end) : ")
  if (ann == "") : break
  annotate = open(filename, 'w')
  annotate.write("\n\n\n\n\n\n" + ann)
  annotate.close()
print "Annotate finished" 

