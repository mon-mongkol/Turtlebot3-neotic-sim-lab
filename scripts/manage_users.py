#!/usr/bin/env python3
"""
Node-RED User Management Tool
- Decrypt (verify) password
- Register new user
- List users
- Reset password
"""

import mysql.connector
import bcrypt
import sys

# DB Config (matches .env)
DB_CONFIG = {
    "host": "127.0.0.1",
    "port": 3306,
    "user": "istdbUser",
    "password": "interface2563",
    "database": "istuvd",
}


def get_connection():
    return mysql.connector.connect(**DB_CONFIG)


# ─── LIST USERS ───────────────────────────────────────────────────────────────
def list_users():
    conn = get_connection()
    cursor = conn.cursor(dictionary=True)
    cursor.execute("SELECT id, email, firstname, lastname FROM users")
    rows = cursor.fetchall()
    cursor.close()
    conn.close()

    print("\n{'id':<5} {'email':<35} {'firstname':<15} {'lastname'}")
    print("-" * 70)
    for row in rows:
        print(f"{row['id']:<5} {row['email']:<35} {row['firstname']:<15} {row['lastname']}")
    print()


# ─── VERIFY PASSWORD ──────────────────────────────────────────────────────────
def verify_password(email, plain_password):
    conn = get_connection()
    cursor = conn.cursor(dictionary=True)
    cursor.execute("SELECT * FROM users WHERE email = %s", (email,))
    user = cursor.fetchone()
    cursor.close()
    conn.close()

    if not user:
        print(f"[✗] User '{email}' not found.")
        return

    # PHP $2y$ → Python $2a$ compatibility fix
    hashed = user["password"].replace("$2y$", "$2a$")
    match = bcrypt.checkpw(plain_password.encode("utf-8"), hashed.encode("utf-8"))

    if match:
        print(f"[✓] Password is CORRECT for '{email}'")
    else:
        print(f"[✗] Password is WRONG for '{email}'")


# ─── REGISTER USER ────────────────────────────────────────────────────────────
def register_user(email, plain_password, firstname, lastname):
    conn = get_connection()
    cursor = conn.cursor(dictionary=True)

    # Check duplicate
    cursor.execute("SELECT id FROM users WHERE email = %s", (email,))
    if cursor.fetchone():
        print(f"[✗] Email '{email}' already exists.")
        cursor.close()
        conn.close()
        return

    # Hash password
    hashed = bcrypt.hashpw(plain_password.encode("utf-8"), bcrypt.gensalt(rounds=10)).decode("utf-8")

    cursor.execute(
        "INSERT INTO users (email, password, firstname, lastname, last_login, role_id) "
        "VALUES (%s, %s, %s, %s, NOW(), 2)",
        (email, hashed, firstname, lastname),
    )
    conn.commit()
    cursor.close()
    conn.close()
    print(f"[✓] User '{email}' registered successfully!")
    print(f"    Name     : {firstname} {lastname}")
    print(f"    Password : {plain_password}")


# ─── RESET PASSWORD ───────────────────────────────────────────────────────────
def reset_password(email, new_password):
    conn = get_connection()
    cursor = conn.cursor()

    cursor.execute("SELECT id FROM users WHERE email = %s", (email,))
    if not cursor.fetchone():
        print(f"[✗] User '{email}' not found.")
        cursor.close()
        conn.close()
        return

    hashed = bcrypt.hashpw(new_password.encode("utf-8"), bcrypt.gensalt(rounds=10)).decode("utf-8")
    cursor.execute("UPDATE users SET password = %s WHERE email = %s", (hashed, email))
    conn.commit()
    cursor.close()
    conn.close()
    print(f"[✓] Password reset for '{email}'")
    print(f"    New password : {new_password}")


# ─── MENU ─────────────────────────────────────────────────────────────────────
def main():
    print("=" * 50)
    print("  Node-RED User Management Tool")
    print("=" * 50)
    print("1. List users")
    print("2. Verify password")
    print("3. Register new user")
    print("4. Reset password")
    print("0. Exit")
    print("-" * 50)

    choice = input("Choose: ").strip()

    if choice == "1":
        list_users()

    elif choice == "2":
        email = input("Email: ").strip()
        pwd = input("Password: ").strip()
        verify_password(email, pwd)

    elif choice == "3":
        email = input("Email: ").strip()
        pwd = input("Password: ").strip()
        firstname = input("First name: ").strip()
        lastname = input("Last name: ").strip()
        register_user(email, pwd, firstname, lastname)

    elif choice == "4":
        email = input("Email: ").strip()
        pwd = input("New password: ").strip()
        reset_password(email, pwd)

    elif choice == "0":
        sys.exit(0)

    else:
        print("Invalid choice.")


if __name__ == "__main__":
    main()
