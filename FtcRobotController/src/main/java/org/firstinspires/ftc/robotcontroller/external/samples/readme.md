
## Caution
＃＃注意
No Team-specific code should be placed or modified in this ``.../samples`` folder.


//比赛队伍的程序不应该放在或改进例程这个名为samples的文件夹里。


Full or partial Samples should be Copied from here, and then Pasted into
the team's folder, using the Android Studio cut and paste commands.
This automatically changes all file and class names to be consistent.

//全部或部分的程序应该从这里被复制，黏贴到队伍（Teammode）文件夹，用android studio的cut和paste功能。
这个功能会自动地改变所有的文件和类名使其一致。

### Naming of Samples
//例程名



To gain a better understanding of how the samples are organized, and how to interpret the
naming system, it will help to understand the conventions that were used during their creation.


//为了使我们得到更好理解例程，并且说明命名方式，帮助我们理解约定当我们这些例程被使用时。


These conventions are described (in detail) in the sample_conventions.md file in this folder.


//这些命名方式将会被仔细的描述在这个sample_convention.md 文件在这个文件夹里。


To summarize: A range of different samples classes will reside in the java/external/samples.
The class names will follow a naming convention which indicates the purpose of each class.
The prefix of the name will be one of the following:
//概括地说:一系列的不同例程将会包含在java/external/samples。这些类的名字表示他们的用途。
  这些名字的前缀将会在下面。

Template:	This is a minimally functional OpMode used to illustrate the skeleton/structure
            of a particular style of OpMode.  These are bare bones examples.


//template(模版)：这个前缀代表最低限度的功能的机器人程序用来阐明特别方式的机器人程序。这些是示例程序的梗概。



Sensor:    	This is a Sample OpMode that shows how to use a specific sensor.
            It is not intended to drive a functioning robot, it is simply showing the minimal code
            required to read and display the sensor values.


//sensor（传感器）： 这是一个示例机器人程序，它会告诉我们怎么使用特殊的传感器。这些程序不是用来驱动运行机器人，
                  它展示了使用传感器需要的知道的东西和使用传感器的功能。


Hardware:	This is NOT an OpMode, but a helper class that is used to describe
            one particular robot's hardware configuration:   eg: For the K9 or Pushbot.
            Look at any Pushbot sample to see how this can be used in an OpMode.
            Teams can copy one of these to their team folder to create their own robot definition.



//harware（硬件）:  这不是机器人程序，而是一个帮助类，用来描述特殊的机器人硬件配置： 比如：给K9或pushbot。
                   看任何和Pushbot有关的例程来看他怎么在机器人程序中应用。
                   队伍可以复制它们中的一部分到自己的队伍文件夹中来创造自己的机器人。



Pushbot:	This is a Sample OpMode that uses the Pushbot robot hardware as a base.
            It may be used to provide some standard baseline Pushbot OpModes, or
            to demonstrate how a particular sensor or concept can be used directly on the
            Pushbot chassis.

//Pushbot：  这是机器人程序的例程用于Pushbot机器人硬件作为基础。
            它能提供一些标准基线Pushbot机器人和程序，或者展示特殊的传感器或概念可以直接用在Pushbot的底盘上。




Concept:	This is a sample OpMode that illustrates performing a specific function or concept.
            These may be complex, but their operation should be explained clearly in the comments,
            or the comments should reference an external doc, guide or tutorial.
            Each OpMode should try to only demonstrate a single concept so they are easy to
            locate based on their name.  These OpModes may not produce a drivable robot. 

//concept(概念)    ：这是一个机器人程序阐明了运行一个特殊的功能或概念。
                    这些可能很复杂，但他的操作可以用指令很简单的解释。
                    否则机器人程序应该借鉴外部的文档、指导、教程。
                    每一个机器人程序应该尝试展示一个单独的概念，这样它们就会更加容易基于他们的名字。这些机器人程序可能不能运行一个能行动的机器人。




Library:    This is a class, or set of classes used to implement some strategy.
            These will typically NOT implement a full OpMode.  Instead they will be included
            by an OpMode to provide some stand-alone capability.

//library(库) ：这是一种类，或者一系列类用来使一些策略生效。
                 这些类不会使某一个机器人程序生效。
                 而是它会包含在一个机器人程序中提供独立地能力。   　


After the prefix, other conventions will apply:

//在解释了前缀以后，其他的命名习惯也提供了：


* Sensor class names are constructed as:    Sensor - Company - Type
* Hardware class names are constructed as:  Hardware - Robot type
* Pushbot class names are constructed as:   Pushbot - Mode - Action - OpModetype
* Concept class names are constructed as:   Concept - Topic - OpModetype
* Library class names are constructed as:   Library - Topic - OpModetype

＊传感器类命名结构是： 传感器名——生产公司——型号
＊硬件类命名结构是：  硬件名——机器人尺寸
＊Pushbot类命名结构： Pushbot－模式－行动名—机器人程序模式（自动程序／手动程序）
＊K9Bot类命名结构  K9bot－模式－行动名－机器人程序模式（自动程序／手动程序）
＊概念类命名结构：  概念名－主题－机器人程序模式
＊库类命名结构：  库名－主题－机器人程序模式