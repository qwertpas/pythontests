import string
import json
import os
import urllib.request

from colorama import init
from colorama import Fore, Back, Style
from os import path
init()

VERSION_FOLDER = os.getenv('APPDATA') + "\\.minecraft" + "\\versions"

print(Fore.GREEN + "           Minecraft Mapping Finder 1.0")
print(Fore.WHITE + "                  Made by DrOreo002" + Fore.RESET)
print(" ")
print("Detected minecraft server version folder at " + Fore.GREEN + str(VERSION_FOLDER) + Fore.RESET)
print(" ")

version = input("> Please type mc version > ")
jsonFilePath = VERSION_FOLDER + "\\" + version + "\\" + version + ".json"
print("     > Found .json manifest file at " + Fore.GREEN + jsonFilePath + Fore.RESET)
print("> Now opening file. Please wait")

dataJson = None

clientMappingUrl = ""
serverMappingUrl = ""

if not path.exists(jsonFilePath):
    print(Fore.RED + "Cannot find json path in " + jsonFilePath)
    exit(0)
    pass

with open(jsonFilePath, 'r') as f:
    dataJson = json.load(f)
    downloadsSection = dataJson["downloads"]

    # Check if available or not
    if "client_mappings" not in downloadsSection:
        print(Fore.RED + "This version doesn't have mappings available")
        exit(0)
        pass
        
    clientMappingUrl = downloadsSection["client_mappings"]["url"] 
    serverMappingUrl = downloadsSection["server_mappings"]["url"]

    print("     > Found " + Fore.GREEN + "client_mapping" + Fore.WHITE + ": " + clientMappingUrl)
    print("     > Found " + Fore.GREEN + "server_mapping" + Fore.WHITE + ": " + serverMappingUrl)
    print(Fore.RESET)

download = str(input("> Should I download the mappings? [Y/N]: "))

if download.lower() == "y":
    print(Fore.WHITE + "     > Downloading...")

    urllib.request.urlretrieve(clientMappingUrl, 'client_mapping.txt')
    urllib.request.urlretrieve(serverMappingUrl, 'server_mapping.txt')

    print(Fore.GREEN + "    > Downloading completed! file is now saved!")
print(" ")
print(Fore.GREEN + "> Closing program....")