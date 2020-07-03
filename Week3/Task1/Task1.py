import requests

result = requests.get('https://www.linkedin.com')
print(result.text)
file = open("URL info.html", "w")

file.write(result.text)

file.close()
