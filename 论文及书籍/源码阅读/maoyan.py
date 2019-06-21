import requests
import json

def main():
    list = get_city_list()

def get_city_list():
    jsondata = requests.get("https://maoyan.com/ajax/cities")
    j = json.loads(jsondata.content)
    # print(j)
    for key, value in j.items():
        if (key == 'letterMap'):
            print(value)
            j2 = json.loads(value)
        #print (key, value)


if __name__ == "__main__":
    main()