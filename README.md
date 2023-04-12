openai_ros
==========

ROS1 wrapper for (OpenAI Python Library)[https://pypi.org/project/openai/].

How to start program
--------------------

Start from sample lamuch file
```
$ export OPENAI_API_KEY="xxxxxxxxx"
$ roslaunch openai_ros openai.launch max_tokens:=256 model:=text-davinch-003
```

or put following code within your launch file.
```
  <group ns="openai">
    <node pkg="openai_ros" type="openai_node.py" name="openai" output="screen">
      <rosparam command="load" file="openai_credentials.json" />
    </node>
  </group>
```
The contents of `openai_credentials.json` file is something like
```
key: xxxxxxxxx
max_tokens: 256
model: text-davinci-003
```

How to use service
------------------

```
$ rosservice call /openai/get_response '{prompt: "Write a poem about OpenAI"}'
```

