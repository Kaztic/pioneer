#!/usr/bin/env python3
"""
Manually create a FoxMQ user by generating argon2id password hash
and adding it to users.toml
"""

import sys
import getpass
import toml
import subprocess
import os

def generate_argon2id_hash(password: str) -> str:
    """Generate argon2id hash for password"""
    try:
        import argon2
        hasher = argon2.PasswordHasher(
            time_cost=2,
            memory_cost=19456,
            parallelism=1,
            hash_len=32
        )
        return hasher.hash(password)
    except ImportError:
        # Try using argon2-cffi
        try:
            from argon2 import PasswordHasher
            hasher = PasswordHasher(
                time_cost=2,
                memory_cost=19456,
                parallelism=1,
                hash_len=32
            )
            return hasher.hash(password)
        except ImportError:
            print("Error: argon2 library not found")
            print("Install with: pip3 install argon2-cffi")
            sys.exit(1)

def main():
    print("FoxMQ User Creator (Manual)")
    print("=" * 40)
    
    username = input("Enter username (default: pioneer_robot): ").strip()
    if not username:
        username = "pioneer_robot"
    
    password = getpass.getpass("Enter password: ")
    if not password:
        print("Error: Password cannot be empty")
        sys.exit(1)
    
    password_confirm = getpass.getpass("Confirm password: ")
    if password != password_confirm:
        print("Error: Passwords don't match")
        sys.exit(1)
    
    # Generate hash
    print("\nGenerating password hash...")
    password_hash = generate_argon2id_hash(password)
    
    # Load existing users.toml
    users_file = "foxmq_cluster/foxmq.d/users.toml"
    users_data = {}
    
    if os.path.exists(users_file):
        with open(users_file, 'r') as f:
            content = f.read()
            # Parse TOML manually since format is [users.Username]
            lines = content.split('\n')
            for line in lines:
                line = line.strip()
                if line.startswith('[users.') and line.endswith(']'):
                    existing_user = line[7:-1]  # Extract username from [users.Username]
                    users_data[existing_user] = None  # Placeholder
    else:
        # Create directory if needed
        os.makedirs(os.path.dirname(users_file), exist_ok=True)
    
    # Append new user to file
    with open(users_file, 'a') as f:
        f.write(f"\n[users.{username}]\n")
        f.write(f"password-hash = \"{password_hash}\"\n")
    
    print(f"\n✓ User '{username}' created successfully!")
    print(f"\nPassword hash: {password_hash[:50]}...")
    print(f"\nUsers file updated: {users_file}")
    print("\nUpdate your launch file with:")
    print(f"  'mqtt_username': '{username}',")
    print(f"  'mqtt_password': '{password}',")

if __name__ == '__main__':
    main()

