#!/usr/bin/env python3
import shelve

session_data_path = "/home/spot/session_data"

with shelve.open(session_data_path) as db:
    for key in db.keys():
        print(f"key: {key}, data: {db[key]}")


