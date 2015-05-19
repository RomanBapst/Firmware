#!/usr/bin/python
from __future__ import division
import sdlog2_dump
import sys, os


class LOG(object):
    def __init__(self,filename):
        self.filename = filename
        self.time = []
        self.roll_rate = []
        self.pitch_rate = []
        self.yaw_rate = []
        self.roll_rate_sp = []
        self.pitch_rate_sp = []
        self.yaw_rate_sp = []
        
        self.time_air = []
        self.roll_rate_air = []
        self.pitch_rate_air = []
        self.yaw_rate_air = []
        self.roll_rate_sp_air = []
        self.pitch_rate_sp_air = []
        self.yaw_rate_sp_air = []
        
        self.landed = []
        
        self.mean_error_roll = 0
        self.mean_error_pitch = 0
        self.mean_error_yaw = 0
        self.max_error_roll = 0
        self.max_error_pitch = 0
        self.max_error_yaw = 0
        
        self.time_max_PR_error = 0
        self.time_max_RR_error = 0
        self.time_max_YR_error = 0
        
        # read log file        
        self.read_log()
        
        
    def read_log(self):
        f = open(self.filename,'r')
        header_list = ((f.readline()).rstrip('\n')).split(',')
        header_dic = {}
        for index,item in enumerate(header_list):
            header_dic[item] = index
            
        for line in f.readlines():
            line = line.rstrip('\n').split(',')
            self.time.append(float(line[header_dic["TIME_StartTime"]]))
            self.roll_rate.append(float(line[header_dic["ATT_RollRate"]]))
            self.pitch_rate.append(float(line[header_dic["ATT_PitchRate"]]))
            self.yaw_rate.append(float(line[header_dic["ATT_YawRate"]]))
            self.roll_rate_sp.append(float(line[header_dic["ARSP_RollRateSP"]]))
            self.pitch_rate_sp.append(float(line[header_dic["ARSP_PitchRateSP"]]))
            self.yaw_rate_sp.append(float(line[header_dic["ARSP_YawRateSP"]]))
            self.landed.append(float(line[header_dic["STAT_Landed"]]))
            
        self.time = [x - self.time[0] for x in self.time]
            
    def get_in_air_data(self):
        
        for index,item in enumerate(self.landed):
            if item==0:
                self.roll_rate_air.append(self.roll_rate[index])
                self.roll_rate_sp_air.append(self.roll_rate_sp[index])
                self.pitch_rate_air.append(self.pitch_rate[index])
                self.pitch_rate_sp_air.append(self.pitch_rate_sp[index])
                self.yaw_rate_air.append(self.yaw_rate[index])
                self.yaw_rate_sp_air.append(self.yaw_rate_sp[index])
                self.time_air.append(self.time[index])
            
        self.time_air = [x - self.time[0] for x in self.time_air]
            
    def get_performance(self):
        # get in air data
        self.get_in_air_data()
        
        max_error_RR_prev = 0
        max_error_PR_prev = 0
        max_error_YR_prev = 0
        
        # get average tracking error and max tracking errors
        for index,item in enumerate(self.time_air):
            self.mean_error_roll = self.mean_error_roll + (self.roll_rate_sp_air[index] - self.roll_rate_air[index])**2
            self.mean_error_pitch = self.mean_error_pitch + (self.pitch_rate_sp_air[index] - self.pitch_rate_air[index])**2
            self.mean_error_yaw = self.mean_error_yaw + (self.pitch_rate_sp_air[index] - self.pitch_rate_air[index])**2
            
            self.max_error_roll = max(self.max_error_roll, abs(self.roll_rate_sp_air[index] - self.roll_rate_air[index]))
            self.max_error_pitch = max(self.max_error_pitch, abs(self.pitch_rate_sp_air[index] - self.pitch_rate_air[index]))
            self.max_error_yaw = max(self.max_error_yaw, abs(self.yaw_rate_sp_air[index] - self.yaw_rate_air[index]))
            
            if self.max_error_roll > max_error_RR_prev:
                max_error_RR_prev = self.max_error_roll
                self.time_max_RR_error = item
                
            if self.max_error_pitch > max_error_PR_prev:
                max_error_PR_prev = self.max_error_pitch
                self.time_max_PR_error = item
                
            if self.max_error_yaw > max_error_YR_prev:
                max_error_YR_prev = self.max_error_yaw
                self.time_max_YR_error = item
            

        self.mean_error_roll/=len(self.time_air)
        self.mean_error_pitch/=len(self.time_air)
        self.mean_error_yaw/=len(self.time_air)
            
    def print_performance(self):
        print "\n Flight Performance Summary:\n"
        print "max error roll-rate:\t %f at %f seconds\n" % (self.max_error_roll,self.time_max_PR_error/1e6)
        print "mean error roll-rate:\t %f\n" % self.mean_error_roll
        print "max error pitch-rate:\t %f at %f seconds\n" % (self.max_error_pitch,self.time_max_RR_error/1e6)
        print "mean error pitch-rate:\t %f\n" % self.mean_error_pitch
        print "max error yaw-rate:\t %f at %f seconds\n" % (self.max_error_yaw,self.time_max_YR_error/1e6)
        print "mean error yaw-rate:\t %f\n" % self.mean_error_yaw
            
            
            
            
        
        
            



def _main():
    file_name = sys.argv[1]
    #only parse log if the csv does not exist yet
    if not os.path.exists(file_name.split('.')[0] + '.csv'):
        sys.argv = [file_name,file_name,'-f',file_name.split('.')[0]+'.csv','-t','TIME','-m','TIME','-m','ATT','-m','ATSP','-m','ARSP','-m','STAT']
        sdlog2_dump._main()
        
    x = LOG(file_name.split('.')[0]+'.csv')
    x.get_performance()
    x.print_performance()
    
if __name__ == "__main__":
    _main()