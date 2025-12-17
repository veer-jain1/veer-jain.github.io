document.addEventListener("DOMContentLoaded", () => {
    const sections = document.querySelectorAll("header, section");

    const observer = new IntersectionObserver(entries => {
        entries.forEach(entry => {
            if (entry.isIntersecting) {
                entry.target.style.opacity = "1";
                entry.target.style.transform = "translateY(0)";
            }
        });
    }, { threshold: 0.15 });

    sections.forEach(section => {
        section.style.opacity = "0";
        section.style.transform = "translateY(25px)";
        section.style.transition = "0.6s ease";
        observer.observe(section);
    });
});
