import csv
import sys
  

def writeDataList(data, name = 'configuration', comments = ['csv Data']):
    """Writes a data list to a CSV file

    :param data: A list of data. Each element in the list will be printed line by line to a CSV file
    :param name: Name of the file to save date data 
    :param comments: List of String comments. Will be written to the begining of the CSV file, as commented lines
        
    """

    try:
        with open( name + '.csv', mode='w') as output:

            for comment in comments:
                output.write("# " + comment)
                output.write('\n')

            for dataLine in data: 
                output.write(','.join([str(x) for x in dataLine]))
                output.write('\n')
            print name +" written to CSV"

    except:
            print "Output file writing error: ", sys.exc_info()[0]

