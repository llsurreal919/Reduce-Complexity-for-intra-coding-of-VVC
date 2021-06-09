# How to use the project

## Build the environment

The following 4 packages and softwares are required.

1. Visual Studio2017
2. Opencv
3. Python
4. Sklearn

## Enable the fast algorithm

1. The related parameters (eg. resolution, frames, QP...) should be setted in file "**encoder_intra.cfg**", which is located in "VVC_project\bin\vs15\msvc-19.16\x86_64\release\\".

   ![image-20210608163137770](C:\Users\hequan\AppData\Roaming\Typora\typora-user-images\image-20210608163137770.png)

2. You should set the parameter "**FAST_ALGORITHM**" (line 55 at TypeDef.h) as 1.

   ![image-20210608163154566](C:\Users\hequan\AppData\Roaming\Typora\typora-user-images\image-20210608163154566.png)

3. You should set the "**width**" and "**height**" (line 832 at EncCu.cpp) as the size of your video. 

   ![image-20210608163259871](C:\Users\hequan\AppData\Roaming\Typora\typora-user-images\image-20210608163259871.png)

4. You should set the "**Py_SetPythonHome**" (line 1140 at EncCu.cpp) as the location of your python environment.

   ![image-20210608163237737](C:\Users\hequan\AppData\Roaming\Typora\typora-user-images\image-20210608163237737.png)

## Print the CU partition

1. You can set the "**CU_SHOW**" (line 63 at TypeDef.h) as 1 to print the CU partition.

   ![image-20210608163322046](C:\Users\hequan\AppData\Roaming\Typora\typora-user-images\image-20210608163322046.png)

2. An example of CU partition contrast between the original VTM and our algorithm is as follows.

VTM:

![image-20210204130514306](C:\Users\hequan\AppData\Roaming\Typora\typora-user-images\image-20210204130514306.png)

Ours:

![image-20210204130536002](C:\Users\hequan\AppData\Roaming\Typora\typora-user-images\image-20210204130536002.png)

## Get the training data

You can also get training data in this project.

1. Set the "GET_TRAINING_SET" (line 59 at TypeDef.h) as 1.

   ![image-20210608163423152](C:\Users\hequan\AppData\Roaming\Typora\typora-user-images\image-20210608163423152.png)

2. Set the "width" and "height" (line 527 at CABACWriter.cpp) as the size of your video. 

   ![image-20210608163442577](C:\Users\hequan\AppData\Roaming\Typora\typora-user-images\image-20210608163442577.png)

3. Four files will be extracted from this project, including "Data_Partition.dat", "Label_Partition.dat", "Data_Terminationn.dat", "Label_Termination.dat".

![image-20210608163518663](C:\Users\hequan\AppData\Roaming\Typora\typora-user-images\image-20210608163518663.png)