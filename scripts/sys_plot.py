import matplotlib.pyplot as plt

def sys_plot():
    fig = plt.figure()
    print("hey there!")
    plt.plot([0, 1, 2, 3])
    plt.show()


# x = np.arange(10)
# y = 2.5 * np.sin(x / 20 * np.pi)
# yerr = np.linspace(0.05, 0.2, 10)

if __name__ == "__main__":
    sys_plot()