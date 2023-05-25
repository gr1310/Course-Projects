from tkinter import *
import tkinter.font as font
from PIL import Image, ImageTk
import os

def register_user():
    username_info=username.get()
    password_info= password.get()

    file=open(username_info,"w")
    file.write(username_info+'\n')
    file.write(password_info)
    file.close()

    username_entry.delete(0,END)
    password_entry.delete(0,END)

    Label(screen1,text="Registeration is Successful", fg="green", font=("Calibri",11)).pack()
    # Label(text="").pack()
    # Button(text="Back to login page", height=1, width= 10, bg="#0077b6",font=("Times",13), command=delete_s1).pack()

def delete_s1():
    screen1.destroy()

def register():
    global screen1
    screen1= Toplevel(screen)
    screen1.title("register")
    screen1.geometry("400x350")
    screen1.configure(bg="#FFFEF2")

    global username
    global password
    global username_entry
    global password_entry
    username= StringVar()
    password= StringVar()
    Label(screen1, text="Please Enter details below ",width="400",height="3",bg="#caf0f8", font=("Times",16)).pack()
    Label(screen1, text="",bg="#FFFEF2").pack()
    Label(screen1, text="Enter Username",font=("Times",13),bg="#FFFEF2").pack()
    username_entry= Entry(screen1,textvariable=username, width=30)
    username_entry.pack()
    Label(screen1,text="Enter Password",font=("Times",13),bg="#FFFEF2").pack()
    password_entry= Entry(screen1,textvariable=password,width=30)
    password_entry.pack()
    Label(screen1,text="",bg="#FFFEF2").pack()
    Button(screen1, text="Register",bg="#d1ffea",height=2, width= 20, command= register_user).pack()
    Label(screen1,text="",bg="#FFFEF2").pack()
    Button(screen1,text="Close window",bg="#ffcccb",height=2, width= 20, command=delete_s1).pack()


def delete_s4():
    screen4.destroy()
def delete_s5():
    screen5.destroy()
def delete_s8():
    screen8.destroy()

def session():
    # delete_s()
    delete_s2()
    global screen8
    screen8= Toplevel(screen)
    screen8.title("Dashboard")
    screen8.geometry("400x400")
    Label(screen8, text="Welcome to the Dashboard").pack()
    Button(screen8,text="Sign out",command=delete_s8).pack()


def login_success():
    session()
    

def password_not_recognized():
    # print("working...")
    global screen4
    screen4 = Toplevel(screen)
    screen4.title("Failed")
    screen4.geometry("150x100")
    screen4.configure(bg="#ffcccb")
    Label(screen4,text="",bg="#ffcccb").pack()
    Label(screen4, text="Incorrect password",bg="#ffcccb",font=("Times",13)).pack()
    Button(screen4,text="Try again",bg="#ff6863",command=delete_s4).pack()

def direct_register():
    register()
    delete_s5()
    
    

def user_not_found():
    # print("working...")
    global screen5
    screen5 = Toplevel(screen)
    screen5.title("Failed")
    screen5.geometry("150x150")
    screen5.configure(bg="#ffdcd1")
    Label(screen5, text="User not found!",bg="#ffdcd1",font=("Times",13)).pack()
    Button(screen5,text="Try again",bg="#ff6863",command=delete_s5).pack()
    Label(screen5,text="",bg="#ffdcd1").pack()
    Button(screen5,text="Register new user",bg="#d1ffea",command=direct_register).pack()


def login_verify():
    # print("working...")
    username1=username_verify.get()
    password1=password_verify.get()
    usernamee_entry1.delete(0,END)
    password_entry1.delete(0,END)

    list_of_files= os.listdir()
    if username1 in list_of_files:
        file1=open(username1,"r")
        verify=file1.read().splitlines()
        if password1 in verify:
            login_success()
            # print("login success")
        else:
            password_not_recognized()
            # print("password has not been recognized")
    else:
        user_not_found()
        # print("user not found")

def delete_s2():
    screen2.destroy()

def login():
    print("Login session started")
    global screen2
    screen2=Toplevel(screen)
    screen2.title("Login page")
    screen2.geometry("400x350")
    screen2.configure(bg="#E5F6DF")
    Label(screen2,text="Please enter details below to login",width="400",height="3",bg="#b6e9f2", font=("Times",16)).pack()
    Label(screen2,text="",bg="#E5F6DF").pack()

    global username_verify
    global password_verify
    username_verify= StringVar()
    password_verify=StringVar()
    Label(screen2,text="Enter your registered Username",font=("Times",13),bg="#E5F6DF").pack()
    global usernamee_entry1
    global password_entry1
    usernamee_entry1= Entry(screen2, textvariable=username_verify,width=30)
    usernamee_entry1.pack()
    Label(screen2,text="Enter your Password",font=("Times",13),bg="#E5F6DF").pack()
    password_entry1=Entry(screen2,textvariable=password_verify, show="*",width=30)
    password_entry1.pack()
    Label(screen2,text="",bg="#E5F6DF").pack()
    Button(screen2,text="Login",width=20,height=2, command=login_verify,bg="#d1ffea").pack()
    Label(screen2,text="",bg="#E5F6DF").pack()
    Button(screen2,text="Close window",bg="#ffcccb",height=2, width= 20, command=delete_s2).pack()


# def_font=font.Font(family='Times')
def delete_s():
    screen.destroy()
#main screen
def main_screen():
    
    global screen
    screen=Tk()
    screen.geometry("400x400")
    screen.title("Sign up/ Login page")
    screen.configure(bg="#caf0f8")
    Label(text="Sign up/ Log in",width="400",height="3", fg="#caf0f8",bg="#04035e", font=("Times",16,"bold")).pack()
    photo = PhotoImage(file="image.png", width="400", height="150")
    new=Label(image=photo)
    new.pack()
    Label(text="",bg="#caf0f8").pack()
    Button(text="Login", height=1, width= 30, bg="#0077b6",font=("Times",13,"bold"), command=login).pack()
    Label(text="",bg="#caf0f8").pack()
    Button(text="Register", height=1, width= 30,bg="#0077b6",font=("Times",13,"bold"), command=register).pack()
    Label(text="",bg="#caf0f8").pack()
    Button(text="Quit", height=1, width= 30,bg="#0077b6",font=("Times",13,"bold"), command=delete_s).pack()
    screen.mainloop()

main_screen()