from numpy import *
from matplotlib.pyplot import *
import os, glob, re #for handling files
import matplotlib.image as mpimg
import matplotlib.pyplot as plt


figure()
figure(figsize=(16,3))
#ig1 = figure(figsize=(16, 4), dpi=1000)
subplot(1,4,1)
### FOR SUCCESS MATRIX USE BELOW
# plot(offsets,success90,'k-o',offsets,success30,'r-x',markerfacecolor='none',mew=2,markersize=12)
# grid('on')
# xlabel('edge lateral offset (m)',fontsize=14)
# ylabel('Success (0=Failure)',fontsize=14)
# legend(['90 degree edge','30 degree edge'],loc='right')
# title('a',fontsize=14)
####FOR EXAMPLE IMAGE USE BELOW
img=mpimg.imread('Figures/1_whipple_vs_Webots_phi0_0.01 rad.png')
imshow(img)
axis('off')
#title('a',fontsize=14)

subplot(1,4,2)
img=mpimg.imread('Figures/2_DR_vs_Webots_phi_0=$0.01 rad.png')
imshow(img)
axis('off')

# figure()
subplot(1,4,3)
img=mpimg.imread('Figures/3_closeloop_bicycle_model_vs_Webots_phi_0=$-0.0 rad.png')
imshow(img)
axis('off')
# legend(['90 degree edge','30 degree edge'])

# figure()
subplot(1,4,4)
img=mpimg.imread('Figures/4_closeloop_motocycle_model_vs_Webots_phi_0=$0.0 rad.png')
imshow(img)
axis('off')

plt.tight_layout()
savefig("Figures/plot.png",dpi=1000)
show()
