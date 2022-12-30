import json


def pairs(l: list) -> list:
    for i in range(len(l) - 1):
        yield l[i], l[i + 1]


def triples(l: list) -> list:
    for i in range(len(l) - 2):
        yield l[i], l[i + 1], l[i + 2]


def read_json(file: str) -> dict:
    with open(file, "r") as f:
        return json.load(f)
