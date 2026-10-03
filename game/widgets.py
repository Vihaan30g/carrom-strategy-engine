"""
ui/widgets.py
==============
Minimal, dependency-free pygame widgets used across every screen.
"""

import pygame

from .constants import BUTTON_BG, BUTTON_BG_HOVER, BUTTON_BORDER, BUTTON_TEXT


class Button:
    def __init__(self, rect, text, font, on_click=None, color=BUTTON_BG):
        self.rect = pygame.Rect(rect)
        self.text = text
        self.font = font
        self.on_click = on_click
        self.color = color
        self.hovered = False

    def handle_event(self, event) -> bool:
        if event.type == pygame.MOUSEMOTION:
            self.hovered = self.rect.collidepoint(event.pos)
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                if self.on_click:
                    self.on_click()
                return True
        return False

    def draw(self, screen):
        color = BUTTON_BG_HOVER if self.hovered else self.color
        pygame.draw.rect(screen, color, self.rect, border_radius=8)
        pygame.draw.rect(screen, BUTTON_BORDER, self.rect, width=2, border_radius=8)
        surf = self.font.render(self.text, True, BUTTON_TEXT)
        tr = surf.get_rect(center=self.rect.center)
        screen.blit(surf, tr)


class Slider:
    """Horizontal drag slider mapping to a [min_val, max_val] float range."""

    def __init__(self, rect, min_val, max_val, value, font, label="", value_fmt="{:.1f}", suffix=""):
        self.rect = pygame.Rect(rect)
        self.min_val = min_val
        self.max_val = max_val
        self.value = value
        self.font = font
        self.label = label
        self.value_fmt = value_fmt   # format spec applied to self.value
        self.suffix = suffix         # e.g. " g", " cm/s" -- appended after the number
        self.dragging = False

    def _value_to_x(self):
        t = (self.value - self.min_val) / (self.max_val - self.min_val)
        return self.rect.x + int(t * self.rect.width)

    def _x_to_value(self, x):
        t = max(0.0, min(1.0, (x - self.rect.x) / self.rect.width))
        return self.min_val + t * (self.max_val - self.min_val)

    def handle_event(self, event):
        handle_x = self._value_to_x()
        handle_rect = pygame.Rect(handle_x - 8, self.rect.y - 6, 16, self.rect.height + 12)
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if handle_rect.collidepoint(event.pos) or self.rect.collidepoint(event.pos):
                self.dragging = True
                self.value = self._x_to_value(event.pos[0])
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION and self.dragging:
            self.value = self._x_to_value(event.pos[0])

    def draw(self, screen):
        pygame.draw.rect(screen, (70, 74, 82), self.rect, border_radius=4)
        handle_x = self._value_to_x()
        pygame.draw.circle(screen, (86, 160, 255), (handle_x, self.rect.centery), 9)
        pygame.draw.circle(screen, (20, 20, 20), (handle_x, self.rect.centery), 9, width=2)
        value_text = self.value_fmt.format(self.value) + self.suffix
        label_surf = self.font.render(f"{self.label}: {value_text}", True, (230, 230, 230))
        screen.blit(label_surf, (self.rect.x, self.rect.y - 26))
