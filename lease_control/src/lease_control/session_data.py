#!/usr/bin/env python3
from pydantic import BaseModel

class SessionData(BaseModel):
    ipfs_hash: str
    extrinsic_hash: str
    state: str