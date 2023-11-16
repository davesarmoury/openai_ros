#!/usr/bin/env python3

from openai_ros.msg import StringArray
from openai_ros.srv import Completion, CompletionResponse
import rospy
from openai import OpenAI

def servicer(req):
    global client, max_tokens, model
    res = CompletionResponse()

    response = client.completions.create(model=model, prompt=req.prompt, temperature=req.temperature, max_tokens=max_tokens)

    res.finish_reason = response.choices[0].finish_reason
    res.text = response.choices[0].text
    res.model = response.model
    res.completion_tokens = response.usage.completion_tokens
    res.prompt_tokens = response.usage.prompt_tokens
    res.total_tokens = response.usage.total_tokens

    return res

def main():
    global client, max_tokens, model
    pub = rospy.Publisher('available_models', StringArray, queue_size=1, latch=True)
    rospy.init_node('openai_node', anonymous=True)

    client = OpenAI(api_key=rospy.get_param('~key'))
    max_tokens = rospy.get_param('~max_tokens', default=256)
    model = rospy.get_param('~model', default='text-davinci-003')

    models_msg = StringArray()
    for m in client.models.list():
        models_msg.data.append(m.id)

    if model not in models_msg.data:
        rospy.logwarn(model + " is not an available model")
        rospy.logwarn(models_msg.data)
        return

    pub.publish(models_msg)

    rospy.Service('get_response', Completion, servicer)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
