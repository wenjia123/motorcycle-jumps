from matplotlib.pyplot import *
import numpy as np
import io

import matplotlib
matplotlib.rcParams.update({'font.size': 24})




class RealTimePlot:
    def __init__(self,buffsize=100,x_label='X data',y_label='Y data',marker='k',delaycounts=10,ylimlow=-1,ylimhigh=1,displayPlot=False):
        #set up the plot
        self.ylimlow = ylimlow
        self.ylimhigh = ylimhigh
        self.DPI = 40
        self.displayPlot = displayPlot
        self.fig,self.ax = subplots(1,1,dpi=self.DPI)
        #self.ax.hold(True)
        self.buffsize=buffsize

        self.xdata=[]
        self.ydata=[]
        self.marker = marker
        self.xlabel = x_label
        self.ylabel = y_label
        self.plt = self.ax.plot(self.xdata,self.ydata,self.marker,linewidth=5)[0]
        xlabel(self.xlabel)
        ylabel(self.ylabel)

        #set up a delay counter so we don't draw every time
        self.delaycounts = delaycounts
        self.delaycounter = 0
        self.img_array = np.array([])
        self.fig.canvas.draw()
        buf = np.array(self.fig.canvas.renderer.buffer_rgba())
        buf = np.swapaxes(buf,0,1)
        self.img_array = buf
        self.imgsize = self.img_array.shape
        self.newPlot = False
        self.fig.tight_layout()

    def update(self,newx,newy):
        self.newPlot = False
        #update the delay counter
        self.delaycounter+=1
        #if we have a buffer, fill xdata and ydata only to buffer
        if(self.buffsize!=-1):
            if len(self.xdata)<self.buffsize:
                #this appends our newest values to our variables of interest.
                self.xdata.append(newx)
                self.ydata.append(newy)
            else: #this means that the buffer needs to lose the oldest value, and gain the newest value.
                #make t equal to the second oldest value to the second newest value
                self.xdata = self.xdata[1:]
                #add the newest value on to the end, maintaining a vector of buffsize.
                self.xdata.append(newx)
                self.ydata = self.ydata[1:]
                self.ydata.append(newy)
        else:
            self.xdata.append(newx)
            self.ydata.append(newy)

        if ((self.delaycounter==self.delaycounts)):#if enough time has passed
            self.newPlot = True
            #reset delay counters
            self.delaycounter=0
            #this sets the line plt1 data to be our updated x and y vectors.
            self.plt.set_data(self.xdata,self.ydata)
            self.ax.set_xlim(self.xdata[0],self.xdata[-1])
            self.ax.set_ylim(self.ylimlow,self.ylimhigh)
            #the draw command is last, and tells matplotlib to update the figure!!
            # self.fig.canvas.draw()
            self.fig.canvas.draw()
            buf = np.array(self.fig.canvas.renderer.buffer_rgba())
            buf = np.swapaxes(buf,0,1)
            self.img_array = buf
            self.imgsize = self.img_array.shape
            print("shape: "+str(self.imgsize))

            if(self.displayPlot):
                #need a tiny pause to actually do it.
                pause(.001)
